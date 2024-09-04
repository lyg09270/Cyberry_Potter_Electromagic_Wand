/*
 * Copyright (c) 2018-2020, Jianjia Ma
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-06-30     Jianjia Ma   The first version
 */
 
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include "nnom.h"
#include "weights.h"

int8_t* load(const char* file, size_t * size)
{
	size_t sz;
	FILE* fp = fopen(file,"rb");
	int8_t* input;
	assert(fp);
	fseek(fp, 0, SEEK_END);
	sz = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	input = malloc(sz);
	fread(input, 1, sz, fp);
	fclose(fp);
	*size = sz;
	return input;
}

nnom_status_t callback(nnom_model_t* m, nnom_layer_t* layer)
{
	float scale = 1 << (layer->out->tensor->q_dec[0]);
	printf("\nOutput of Layer %s", default_layer_names[layer->type]);
	for (int i = 0; i < tensor_size(layer->out->tensor); i++)
	{
		if (i % 16 == 0)
			printf("\n");
		printf("%f ", (float)((int8_t*)layer->out->tensor->p_data)[i] / scale);
	}
	printf("\n");
	return NN_SUCCESS;
}

#ifdef NNOM_USING_STATIC_MEMORY
uint8_t static_buf[1024 * 500];
#endif
 
int main(int argc, char* argv[])
{
	FILE* fp;
	nnom_model_t* model;
	nnom_predict_t * pre;
	int8_t* input;
	float prob;
	uint32_t label;
	size_t size = 0;

	input = load("test_data.bin", &size);	// load a continuous input dataset (test bin)
	fp = fopen("result.csv", "w");			// csv file for result
	fprintf(fp, "label, prob\n");				// header of csv
	printf("validation size: %d\n", (uint32_t)size); 
	

#ifdef NNOM_USING_STATIC_MEMORY
	// when use static memory buffer, we need to set it before create
	nnom_set_static_buf(static_buf, sizeof(static_buf)); 
#endif

	model = nnom_model_create();				// create NNoM model
	pre = prediction_create(model, nnom_output_data, sizeof(nnom_output_data), 4); // mnist, 10 classes, get top-4
	//model_set_callback(model, callback);
	
	// now takes label and data from the file and data
	for(size_t seek=0; seek < size;)
	{
		// labels
		uint8_t true_label[128];
		memcpy(true_label, input + seek, 128);
		seek += 128;
		// data
		for(int i=0; i < 128; i++)
		{
			if(seek >= size)
				break;
			memcpy(nnom_input_data, input + seek, sizeof(nnom_input_data));
			seek += sizeof(nnom_input_data);
			
			//nnom_predict(model, &label, &prob);				// this will work independently
			prediction_run(pre, true_label[i], &label, &prob);  // this provide more infor but requires prediction API
			
			// save results
			fprintf(fp, "%d,%f\n", label, prob);
		}
		printf("Processing %d%%\n", (uint32_t)(seek * 100 / size));
	}
	
	// save result
	fclose(fp);

	// print prediction result
	prediction_end(pre);
	prediction_summary(pre);
	prediction_delete(pre);

	// model
	model_stat(model);
	model_delete(model);

	free(input);
	return 0;
}
