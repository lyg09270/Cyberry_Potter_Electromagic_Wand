#include "nnom.h"

/* Weights, bias and Q format */
#define TENSOR_CONV1D_KERNEL_0 {23, -73, -15, -45, -42, -62, -27, -17, 8, 12, 26, 62, -24, 74, 29, -38, -41, 14, 60, 16, -35, -10, -17, 17, 43, -66, 18, -14, 78, -16, 24, 69, -5, 8, -19, 42, -46, -50, -60, -15, 19, 25, -7, -49, 47, 37, -52, 13, 76, 20, -12, 86, -33, -2, -74, -41, -36, 22, -57, -13, -46, -29, 50, 67, 2, -65, -38, -78, 5, 39, -81, 9, -23, -38, -12, -75, -55, 47, -76, 29, 23, 61, 64, 58, 26, -26, 78, 83, -37, -3, 9, -24, -30, 21, 38, -68, -4, 32, -10, -52, 2, 72, -59, -12, -11, -50, 13, 67, -29, 44, -66, -36, -46, -69, 13, 53, -6, -54, 68, 31, -42, 57, -4, -36, 6, -69, -68, -9, 37, -6, 15, 30, -14, -23, 84, 29, 23, 45, 75, -34, -61, 34, -30, 38, 58, 19, -58, 39, -61, -75, 19, -28, 18, -48, 62, -70, -65, -29, -41, 8, 62, -95, 53, 6, 23, -21, 39, 98, 35, 46, 81, -65, -61, 73, -13, -21, 25, -19, -51, -12, 59, 36, -11, 80, -50, -23, 70, 46, 42, 8, -20, -63, 15, -43, -54, -49, 26, -93, 36, -28, -40, -50, -10, 4, 27, -33, -78, -59, -29, -2, -77, -28, -6, -53, 48, -19, -14, 63, -7, -60, -18, -50, 57, -29, -84, -25, 54, 66, -66, 29, 42, -74, -35, -42, -54, -72, 44, -51, -28, -61, -50, -59, 15, -20, -51, 44, -32, 15, 94, -39, -12, 54, -54, 11, -45, -1, 29, -51, 8, 24, -20, -6, -42, 9, 82, -49, 33, 79, 2, 31}

#define TENSOR_CONV1D_KERNEL_0_DEC_BITS {8}

#define TENSOR_CONV1D_BIAS_0 {-22, -4, 73, 59, 83, -39, 22, 54, -22, 39, -30, -1, 33, 30, 34, 58, 76, -41, 59, 61, -4, -48, 31, 68, -29, -28, -5, 21, -20, -18}

#define TENSOR_CONV1D_BIAS_0_DEC_BITS {9}

#define CONV1D_BIAS_LSHIFT {3}

#define CONV1D_OUTPUT_RSHIFT {8}

#define TENSOR_CONV1D_1_KERNEL_0 {-21, -43, 52, -16, 59, -30, 4, 59, -48, -17, -2, -22, -35, -53, -45, 29, 64, -48, -25, 75, -14, -45, 54, 49, -52, -18, 9, 5, -52, -27, 52, -38, 45, -16, 41, 38, 30, 35, 64, 55, -70, 53, -8, -29, -1, 26, 61, 39, 27, 87, -51, 18, 3, 13, -5, -52, 48, -38, -4, -37, -22, -2, 31, 15, 46, -20, 60, 57, -15, 31, -58, 10, -14, 37, 42, 27, -6, 5, 20, 47, 24, -1, 25, -8, 21, 0, 45, 14, -53, 48, 0, 64, 11, 59, 76, -4, 46, 11, 43, -51, -10, 9, 51, 24, 60, 25, 39, -18, 50, 72, 10, 0, 7, 32, -62, 11, 13, 14, 1, -1, -28, -10, 53, 47, 57, -69, -22, -11, -31, -28, 37, 11, 17, 0, 38, -41, 16, 4, 10, 11, -65, -42, -53, 62, -30, 42, -23, 34, -16, 25, -26, 9, -40, 74, 8, -24, -13, -54, 49, -63, 11, -19, -30, 63, 7, -51, -53, 5, -13, 11, -58, 0, -5, 9, -68, -2, -1, -24, -15, -77, 15, 7, 28, 25, 63, 33, -18, 79, 17, 55, 79, 27, 51, -4, 52, 0, -3, 30, -65, 52, 20, 49, -22, 56, 20, 2, -6, 61, 45, -16, 59, -28, 26, -25, 34, 70, 53, 14, 62, -49, -12, -36, 15, -4, 32, -2, 43, -5, -66, 6, 36, 64, 58, 10, 45, -21, 51, 29, -31, -17, 32, -45, 4, 2, 16, 22, 10, 63, 11, -36, 71, 2, 1, -36, -34, 20, 68, 21, -57, 17, -24, 12, -28, 31, 58, 41, 58, 48, -33, 50, -57, -15, 64, 34, -32, -8, -53, 24, -63, 30, 2, -22, 37, 16, 4, 67, 15, -20, -10, -17, 74, -60, 8, -19, 49, -27, -32, 35, -16, -7, 14, -17, 44, 18, 45, 54, 10, 37, -52, 47, 28, -8, 48, 46, -11, 66, 40, 37, -3, -35, -24, -18, 5, 56, 13, 20, -43, -40, 44, 66, -53, -49, 54, 53, 61, -29, -23, 66, -46, -6, 36, -48, -19, 63, -28, 66, -8, -21, 19, -22, 8, -24, 3, 30, 8, -13, -26, 4, -28, 24, 36, -46, 11, 27, -52, 48, -14, 14, -28, 4, 21, -2, -13, -41, -60, -21, -32, 55, -47, 10, 57, 71, 57, -34, 8, 52, -9, -28, -18, -4, -17, -21, -7, 41, -27, -10, 17, 38, -27, -42, 7, -47, 10, -16, -34, -47, -3, 78, 8, -21, -45, 41, 13, 27, -3, 34, -45, -52, -23, 43, -16, -30, -56, -7, 34, 50, -51, -8, 46, 41, 74, -7, 14, 45, -33, 50, -35, 74, -50, 27, -25, 64, 1, 40, 12, -31, -9, -72, 8, -45, 55, 66, -22, 0, 4, -9, 47, -28, 69, 36, -37, 71, 37, -3, 68, -5, -45, 3, 24, 57, 6, -20, -25, 1, 42, 51, -4, -9, 9, 37, -36, 9, -40, -35, 32, -27, -11, -34, 25, -40, 18, -1, -20, 62, 72, 7, -36, -44, 43, 65, 0, 2, -52, -23, 14, 14, 61, 40, -26, -5, -29, -22, 48, -26, 10, -45, 48, -31, 40, -27, -56, 56, -4, 6, 40, 36, -39, 5, 56, -22, -41, 51, -10, 0, -32, 4, -7, 67, -8, 44, -52, 10, 46, -39, -1, -24, -26, -33, -39, -21, 7, -17, -15, 26, -53, 27, -11, -30, 1, -47, -28, -6, -6, -47, -55, -36, 0, -65, -10, -3, -63, -33, 77, -18, -58, 59, 12, -8, 30, 69, -38, -64, -44, -39, 18, 8, -9, 36, 15, -27, 62, -51, 20, 11, -12, -51, -3, 10, 19, 29, -56, 9, 57, -20, -58, 73, 10, 30, -62, -6, -15, 24, -65, -54, -68, 57, 67, 38, -16, -14, -27, -54, -28, 5, -52, -4, -49, -13, -23, 59, -2, 30, 21, 26, -9, 37, 52, -24, 67, 15, 26, 57, -34, 22, -18, 33, -11, 49, -51, 8, 16, 5, -30, 66, -15, 9, 40, -23, -22, -31, 11, -16, -28, -51, 28, -47, 14, -69, -43, -27, -4, -14, -6, -36, 47, 11, -66, -13, -37, -27, 11, 7, -11, 38, 21, -36, 19, -32, 51, 7, -36, 14, 29, -31, -44, 25, 54, -40, -18, 8, 20, 41, 36, -29, -31, 27, 7, 52, -36, 38, -57, -38, -4, 35, 33, 24, 50, -52, 11, 40, -26, 23, 41, -53, -76, 6, -50, -55, 4, -24, 11, -53, 14, -50, -45, 58, 43, 4, -42, 18, -27, 66, -34, -35, 44, 43, 28, -69, 24, 30, -42, -33, 12, 23, -6, 30, 8, -61, 3, -39, 42, 11, 56, 49, -8, 18, 76, 71, -60, -21, 17, 24, -4, -1, 2, -30, 47, 3, -13, 28, 53, 12, -38, -28, -47, 31, 37, 37, -45, -21, 70, 35, -25, 28, 11, 13, 33, 41, -44, 3, 33, 10, 73, 40, 33, -10, -34, -70, 72, -8, -17, 6, -53, -8, 30, -4, 20, 9, 37, -40, -40, -34, -22, 66, -44, -12, -50, 44, -62, 1, -23, 34, -42, 65, 52, 57, 20, 16, 41, 25, -42, -15, -8, -23, -30, 18, 22, -60, 62, -50, 4, 55, 43, 42, 6, -18, -32, 63, -26, 10, -36, 12, -17, -12, 27, 44, 13, 39, 53, 46, 9, 12, -9, -25, 61, 7, 40, -30, 50, -32, 33, -19, 28, 55, -21, 0, -10, -10, -65, -25, -16, -10, -34, -37, 23, 6, -6, -62, 58, 55, -26, -44, -58, 37, -49, -1, -29, -28, 31, -23, -18, -4, 46, 44, 37, -1, 27, 61, -21, -46, -61, 25, -42, 55, 42, 18, -1, -22, -15, -6, 21, 20, -70, -30, -18, 11, -36, 25, -35, -2, 40, -13, -33, 61, 7, -41, 4, -1, -35, -59, -49, 48, 31, 3, 31, -1, 53, 40, 3, -41, 16, 12, -66, -46, 36, 13, 30, 11, -12, -30, -1, -5, -3, -4, 35, -9, -21, 45, -4, 54, -64, -24, -36, -15, -45, 34, -20, 38, -2, -4, 56, 25, 10, 70, -20, -43, 46, -33, 22, 53, -46, 43, -13, -11, 29, 34, -22, 8, 77, 67, -42, 8, 40, -6, -7, 23, -12, 1, 6, 63, 14, 48, -12, 1, -8, 61, -31, -20, -51, 8, 9, 25, 11, -10, 46, 17, 29, -29, 17, 4, -29, -52, 6, 41, 25, -11, -25, 29, 46, 73, 28, 72, -23, 55, -23, 57, -42, 4, 14, 6, 32, 39, 24, -39, 67, -35, 7, -37, 66, 21, 12, -59, -19, -56, 39, -6, -15, 70, 19, 22, 61, -75, -3, -33, -38, -45, 61, 11, -50, -69, -6, -75, 43, 41, -55, 23, 37, 78, -10, -43, 39, 82, 78, -14, 70, -67, 47, -53, 1, -29, -28, 17, 50, 42, 4, -8, -37, 31, -72, 2, 5, -3, 4, 1, -33, 24, 48, -25, -32, 19, -37, 55, -7, 13, -7, -78, -31, -59, 74, -56, -10, -66, 47, -37, 6, -6, -35, -21, -35, 1, 52, 7, 22, 48, -43, -7, 8, 42, 38, -47, 7, -3, 81, -17, 60, 27, -55, -16, 64, -52, -36, 41, 52, 14, -7, 64, 21, 67, -30, 7, 48, -32, 9, 1, -16, -7, 56, -34, 28, -10, 72, -18, 61, 31, 0, -45, -46, -36, -28, -20, -34, 40, 43, 32, -74, 30, -52, 38, 29, 43, 4, -47, 15, 26, -60, 12, -14, 10, 17, -64, 56, -3, -24, -9, 11, -4, -9, -7, -28, 76, -33, -27, 51, -7, 28, 58, -45, 52, -14, 34, 28, -30, -46, -18, -24, 39, -5, 21, -20, 29, 48, -2, -46, -9, -41, -25, 29, 16, 22, 72, -44, 57, 21, 16, -33, -52, -59, 8, -58, -20, -43, -30, -8, 33, 39, 47, 1, -12, 70, -40, -39, -62, -2, -42, -33, 39, -38, 69, 7, -31, -58, 11, 43, -8, -22, -17, -70, -68, -8, 60, 21, -29, -29, -12, 49, 37, -58, -63, 5, 30, 53, 28, 21, -65, -64, -33, -35, 0, -54, 20, -5, 24, 16, -20, -6, -18, 24, -42, 1, -11, 18, 49, -19, -11, 46, -34, -48, -37, 74, 66, 41, 24, -8, -41, -32, 28, -40, 8, -44, -39}

#define TENSOR_CONV1D_1_KERNEL_0_DEC_BITS {8}

#define TENSOR_CONV1D_1_BIAS_0 {65, 62, -1, 74, -37, -26, 23, -10, -39, -33, -19, 32, 37, 6, 26}

#define TENSOR_CONV1D_1_BIAS_0_DEC_BITS {9}

#define CONV1D_1_BIAS_LSHIFT {3}

#define CONV1D_1_OUTPUT_RSHIFT {10}

#define TENSOR_DENSE_KERNEL_0 {5, -20, 30, -15, -11, -19, 42, -44, 39, -36, -7, 3, 18, -8, -10, -5, 14, -34, 9, 3, -39, -52, 21, -51, 14, -14, -14, -15, 23, 6, -3, 1, -17, 4, 63, -3, 15, -42, -49, 6, 17, -15, -42, 26, -26, 10, -58, -29, 10, -47, 1, 29, -53, -42, -31, -10, -4, -8, 45, -7, 3, -54, -4, 19, 22, 46, -15, 7, 36, -26, 26, 6, 35, -29, -21, -28, -7, -9, -17, -11, 7, -13, -25, -20, -43, -26, 16, 11, -11, 36, -59, 8, 33, 33, -59, -14, 30, -34, -16, 19, 10, -17, 34, 33, 44, 36, -7, -8, 8, 17, -31, 0, -25, 27, -13, -28, -17, -14, -23, -8, -46, -3, 5, -12, -8, 22, -16, 17, 21, -67, -15, 8, -19, -58, -22, 15, 2, 25, -2, 13, -32, 33, -62, -19, 0, 21, -39, -25, 33, -32, -35, -14, 26, -27, 51, 13, 12, -51, -30, -7, -13, -29, -15, -47, -41, 7, -49, -40, 8, 60, -26, 19, 2, -24, -9, 3, -52, 6, 30, -42, -32, 17, -23, 1, -9, -34, 40, -12, -31, 15, -1, -65, -49, 33, 40, 23, -2, -5, -18, -38, -47, -51, 11, 8, -28, -26, 50, 7, 18, 6, 6, -26, -35, -19, 17, -2, -33, -38, 11, -4, 1, -24, 12, -16, 0, 22, -9, -22, -18, -43, -3, 2, -61, 0, -31, -8, -28, -26, 28, -12, 2, 22, 15, -9, -10, 11, -6, -8, 55, -48, 18, -15, -15, 9, 51, -29, -20, -31, 15, 24, 30, -18, -11, 2, -3, 1, -15, -8, -11, -17, -40, 23, -7, -7, -24, -10, 21, 13, -50, -43, 52, -32, -22, -27, 0, -10, 7, -6, -41, -7, -7, -3, -45, -36, 19, 16, -19, -6, 6, -17, -20, 25, 5, -59, -12, 22, 2, -1, -30, 34, -26, 36, -12, -15, -6, -41, 4, -38, 5, 3, -20, 21, 12, -26, 17, 40, 23, 22, -55, 12, -56, -38, 24, 4, -20, -17, 7, 4, -45, -16, 14, 3, -45, -29, -5, -22, 6, -44, 4, 21, -27, -15, 7, 35, -32, 13, 33, 3, 22, -61, -3, -19, -31, 17, 30, -2, 3, 11, 12, -16, -58, -7, -5, 4, -16, -19, -32, 17, 10, 41, 27, 7, 4, 19, 21, -5, -50, -18, 43, 27, -7, 24, -26, 22, -35, -15, 1, -34, -30, 23, 26, -27, -42, -60, -31, -32, -59, 16, 13, 0, -7, 6, -20, 6, 53, 18, -30, -10, 0, 7, 19, -23, -20, -16, -10, 14, 5, -29, 14, 19, -32, -13, -28, -27, 10, -6, 47, -23, 20, 12, -45, -10, 6, -35, 9, 18, 27, 29, 20, 5, -7, 39, 8, 4, 22, 30, -6, -28, 31, -26, -26, -29, 4, -34, 17, 44, -31, 32, 11, -23, 42, -32, -18, -12, -18, -5, -24, -15, -29, -5, 19, -36, -23, 25, 3, 7, -33, 12, -4, 25, 12, 3, 7, 21, 36, -19, -29, -63, 6, -39, -31, -19, 35, 13, 36, -3, 8, 14, 3, -46, -25, 9, -47, -36, -28, -11, 3, 2, -34, -9, 12, 1, -38, -15, 17, 13, -39, 2, -24, -1, 38, 5, -65, 16, -12, -25, -31, -21, 38, -17, 17, -18, 19, 13, -42, 30, 10, -34, 7, 3, 46, 21, -24, 20, -32, -25, -16, 42, -16, -8, -32, -20, 19, 32, -6, 10, 3, -35, -14, -28, -33, 5, 26, -17, -42, 47, 6, -16, 17, -40, -29, 8, 9, -59, -49, -9, 10, -23, -6, -25, -21, 12, -10, -12, -13, -16, 45, -9, 13, 12, -4, 17, 25, 3, -20, 13, -22, 26, -39, -48, -38, -21, 14, 43, -13, 21, -47, -39, 22, -27, 4, 13, 25, 10, 15, 23, 18, -36, 42, -13, -12, 15, 17, 22, -15, 20, -18, -13, -4, -29, -56, 24, 31, -63, 0, -13, -40, 15, -3, -10, -50, -62, -4, -26, 46, -16, 27, -19, -8, 15, 11, -34, 1, 11, 43, -5, -15, -26, 11, -11, 11, 37, -23, -25, -23, -8, -12, 42, 19, -1, 33, -54, -19, -4, 4, 28, 13, 1, -49, -7, -45, -38, -21, 27, -5, -17, -26, -49, -48, -39, -39, 12, -9, -30, -34, -67, -10, 2, 11, -21, 19, 2, 5, -15, -11, 6, -2, 31, 39, 3, -26, 12, 14, 15, -14, 8, -34, 3, 11, 10, -9, 7, -15, -22, -27, 12, -9, -5, 1, 32, -36, 32, 19, 16, -35, 5, -2, 27, -14, -26, -18, -12, -15, -49, -51, 33, -19, 6, 20, 7, 0, -14, -46, -18, 30, -13, -32, -49, -11, -30, 32, -34, -15, 13, 16, -27, 8, -59, 28, -19, 22, 7, 2, -37, -7, -14, 15, 18, -35, -44, 48, -15, -28, 18, -22, -41, -26, -10, -9, 21, -23, 7, -56, 1, -37, -46, 15, -22, 25, -41, 9, -19, 41, -48, -18, 33, 4, 35, -40, -10, 15, -36, -15, -26, -12, 10, 33, -4, -11, 22, -8, 15, 8, -23, -17, 39, -4, -22, 1, -52, 42, -20, -45, 24, 29, 15, -23, -35, 19, 9, -1, -53, -1, 0, -20, 6, -20, 8, 9, 9, -12, 3, -25, -20, -11, 18, -7, -5, 59, -34, -38, -4, -44, 18, -27, -25, 26, -43, -12, -49, -66, 26, -8, -25, 2, -48, -18, 50, -28, -42, -31, -29, 11, -22, -7, -38, 62, -20, -50, 23, 40, 6, -4, -41, 0, 24, 43, -23, 3, -56, 23, -3, 2, 2, -18, -36, -12, -26, -25, -46, -13, -47, 32, -46, -64, 37, 11, 3, -12, 32, -39, -13, -29, -26, -26, -37, 14, -26, 17, -20, 3, 35, -28, 18, 35, 9, -42, 7, -28, -46, 39, -4, 14, -6, 35, -60, -13, 36, 7, -12, 19, -19, 3, -9, 25, -10, -41, 35, -21, -2, -9, -23, 5, -29, 46, -43, 47, 21, 37, 4, -33, -3, 41, -50, 3, -4, -51, 32, 35, -70, 40, 25, 4, 9, -24, 14, -18, 31, -30, 31, -13, 12, 17, 2, 1, 18, 4, -36, 12, 10, -76, 1, -27, 9, 18, -62, 43, -28, -9, -50, 30, 25, -10, -38, 27, -29, -43, -44, 8, -12, 6, 11, -11, -56, -24, 27, -8, 14, -16, -33, -14, -11, 27, 2, 3, -39, -27, -50, 19, -57, -28, -42, -54, 17, 13, -59, -16, 6, 9, -54, -61, -12, 10, -3, -5, -42, 18, -26, -15, -15, 20, 30, -30, 8, 43, 55, -22, 5, 38, -37, -44, 46, -58, -6, -16, -6, 10, -43, 21, -57, -63, -9, -50, 25, -31, -30, -43, -70, -4, -34, 24, -34, -42, 5, 24, 30, -7, -6, 19, -25, -1, 24, -14, -8, -39, -20, -63, -10, -6, -24, 22, -5, 25, 0, 9, -23, -2, 35, -7, 1, 6, -26, -31, 0, -7, -4, -65, 25, 30, -39, 21, -38, 15, -5, 38, 7, -14, 28, -29, 30, -24, 35, -51, -24, 4, 28, 15, -18, -4, -3, -54, -16, 25, -9, -22, 16, -39, -16, -13, -46, 4, -11, -14, 1, -3, 38, -6, 12, -13, -52, -28, -19, -45, 38, 14, -53, -33, -36, 22, 19, 1, 6, 16, 10, 8, -42, 52, -6, -17, 14, 0, -20, 15, 46, -11, 6, -41, -36, -18, -3, -10, -41, -11, 16, -42, -26, -46, 8, 34, 20, -19, 1, -14, -14, -21, -23, -48, -33, -52, 33, -38, 19, -37, -19, 5, 11, 0, 36, -24, 10, 12, 9, -23, -32, 22, -38, 10, 19, 7, -28, -59, 8, -23, 24, 11, 28, -5, -33, 52, 40, 10, -38, 24, 37, 0, -21, 20, -3, -14, 10, -29, -9, 20, 37, -58, -2, 37, 17, 1, -31, -5, 13, -50, 21, -1, 25, -35, -25, 30, 3, -24, -5, -28, -12, 30, 11, 10, 13, -14, 29, -25, 28, 42, -18, -47, -20, -12, 12, 6, 21, 43, -16, 14, 6, 5, -13, -7, -28, -9, 2, 10, -33, -39, -18, -11, -27, 3, 16, 10, 7, -24, -22, -24, -30, 36, 8, -2, -17, -22, 4, 8, 7, -9, -6, 4, 23, 2, -49, -4, -3, 1, -41, -40, 30, 12, 10, -22, 2, 13, -46, -46, -39, -25, 13, 25, 22, 18, 7, -3, 17, 15, 34, 9, -37, -40, 6, 9, -1, 22, -28, 65, -12, -15, -3, 18, -5, -46, 31, 2, -10, 32, 16, -17, -15, 18, -2, -31, 24, -46, -32, -50, -30, -23, -27, -18, 40, -9, 24, -51, 18, -13, -12, 20, -4, -30, -22, 6, -23, -14, -28, 3, 19, -10, 5, -1, 10, -7, 10, -12, 11, 1, 4, -2, -39, -43, -1, 63, -26, 8, 19, 51, -15, 31, 20, 13, -32, -8, -9, -30, -21, 24, 17, 10, -20, 12, 10, -74, -21, -42, -24, 39, 22, 7, 36, 2, 29, -15, 39, 22, 17, -13, 25, 24, 28, 23, 32, 7, 4, -17, -37, 26, 15, -3, -16, 34, -24, -17, -8, 14, -3, -1, 13, 14, -39, -13, -40, 3, 13, -6, -13, -21, -38, -35, -50, 42, -28, -19, 38, -7, 4, 23, -19, 8, -34, -22, 27, -22, 6, -28, 54, -2, 12, -44, 9, -32, -54, -19, -19, 1, 27, 13, 21, -38, 13, 11, -34, 9, 25, 17, -25, -20, 26, -8, -22, -31, -22, -27, 30, -14, -7, -15, 5, -7, 19, -10, 34, -47, -9, -41, -15, -42, 13, 0, 11, -2, 13, -46, 28, -41, -25, -32, 33, 23, -20, -17, -3, 16, 1, 21, -12, 28, 5, -24, -20, -2, 15, 0, -12, -10, -15, -7, 0, -4, -31, 25, -12, -4, -45, -8, -45, 24, -21, -53, -9, 18, -24, 42, -38, 38, -16, -7, 6, -19, 29, 2, -22, -15, -8, -28, -22, -6, -3, -27, 43, -7, -23, -7, -21, -33, 11, 40, -35, -27, -19, 27, -10, -3, 0, 21, -14, 21, 39, 17, -29, 18, 30, -9, -20, 24, 7, 6, 28, 3, 7, -26, -28, 17, 22, 21, 34, -28, 20, 49, 6, 8, -21, 25, 58, 24, -29, -40, 32, -1, -32, -31, 6, -2, -6, -15, -29, -28, 19, 11, -28, -17, 41, -52, -18, 6, 27, -21, -23, -21, -1, 10, 50, -31, -14, -38, 6, 0, 18, -24, 11, -13, 0, -21, -22, 4, -8, -15, -5, 18, -25, -43, -12, -1, 17, 44, -18, -30, 51, 32, -17, 17, -19, -16, -27, 4, 16, 20, 51, -20, -52, -7, -25, 52, 4, -5, 5, -16, 15, -15, -13, 30, -24, -9, 5, 52, 14, -27, 10, -29, 9, -39, -5, -25, 19, -47, -22, 14, 4, -46, 14, -31, 56, 36, 25, 0, 12, 38, 20, 8, -4, 10, -35, -5, -32, -38, 15, -6, 31, -43, -11, 4, -4, 9, 5, 31, 22, 2, 15, -41, 38, -15, 8, -54, 11, -37, 5, -49, 20, -17, 13, 21, 31, -31, 34, 34, -32, -46, -12, -60, -2, -34, -2, 16, 30, 2, 6, -22, -1, -19, -44, 13, 23, 3, 2, -32, -17, -10, -34, 0, -8, 44, 7, 18, 10, 22, 2, 19, 8, 14, -28, 19, -3, -58, -30, 12, 22, 48, -27, -39, 49, -32, 31, -46, 26, -46, -17, 3, 9, -17, 3, 8, -36, 18, 20, -32, -36, 16, -1, 49, -17, 46, -18, -15, -3, 23, -16, -9, 11, -4, -53, -31, 11, 9, -42, 24, 8, 44, 6, -28, -23, -39, -41, 31, -8, -7, 23, -22, 9, 14, -16, 20, -32, 21, -14, -9, 34, -30, 22, -12, -11, -8, 39, -33, 20, 6, 24, 22, 36, -15, -35, 29, -34, -35, 20, -29, -11, 47, -22, -27, -24, 5, -18, -17, 2, -1, -48, -24, 10, 0, -9, -14, -10, -30, 66, 9, 2, -34, -8, 43, -1, 22, -26, -19, -9, -36, -9, 28, -20, -4, -31, -18, 23, -47, 12, 31, -13, -18, 37, 23, 13, 13, 7, -21, -1, 21, -5, 3, -20, 26, -51, -15, -33, -8, 22, 14, 34, -13, -29, -1, -47, -36, -9, 8, -40, 2, -35, 19, 16, 26, 64, 28, -5, 15, -18, -19, -18, 10, -64, -21, -39, 44, 10, -27, -6, -15, 6, -13, 22, 5, -31, -29, -3, 34, 3, -11, -43, -16, -5, -28, 16, -16, 53, -26, -19, -4, 50, -17, -2, 13, -19, 12, -41, 20, -16, 0, 34, -1, 15, -38, -46, 8, -26, 12, -12, 52, -10, -21, -1, 11, 32, -31, 23, -16, -26, -28, -36, 39, 20, 25, -1, -3, 0, -6, 42, 30, 17, -8, 0, -25, 25, 30, 17, -17, -18, 8, -20, 4, -11, -19, 4, -24, 8, 40, -18, -5, -57, -8, 24, 51, -57, -12, -29, 3, -18, -60, -22, 5, -1, 31, 10, 2, 20, 55, -8, 13, 11, -43, -41, -23, 35, -54, -37, -20, 48, -11, 1, -17, -26, 7, 20, 51, 17, -17, -21, -31, -48, 11, 30, -6, -69, 12, 33, -25, 31, 8, 26, 7, 37, -35, -37, -14, -11, 50, -4, -15, -57, -53, 5, 23, -48, -53, -42, -11, -3, 14, 6, -20, 43, 0, 14, -22, 38, -41, 1, 13, -16, -8, 6, -5, -2, -6, -10, -34, -20, 27, -41, -6, -28, -30, -42, -5, -39, 14, -12, -16, 37, 14, 18, 1, 31, 2, 38, 33, -19, -25, 11, -29, -22, -1, -15, -4, 10, -15, 7, -31, -14, -24, 16, -37, -10, -23, -38, -16, -22, 27, -14, 14, -25, -47, -25, -15, -39, 23, -51, -15, 13, -6, 35, -30, -20, 28, -14, -1, 2, 7, -18, -8, -36, 16, 21, 8, 34, 39, -23, 29, -33, 10, 29, -24, -19, 13, 15, -34, 19, -30, -33, 34, -54, 13, 47, -36, -3, -22, 36, -51, 9, -12, -19, 21, -3, -7, -29, 30, -30, -3, -13, -58, 60, -30, -20, -10, -10, -31, -19, -29, 5, -5, -5, 21, -6, 9, -21, -21, 0, -24, 3, 11, 30, 4, 37, 7, -15, 19, 39, -15, 34, -42, 6, 35, -23, 19, 43, -9, -17, 30, 4, -21, -16, 35, -6, -7, 6, -38, -12, 32, 7, -10, -29, -42, -27, -6, -53, -18, 32, -12, -19, 9, -27, 43, -23, 18, 15, -20, -10, -2, -10, 27, 15, 38, -19, -56, 13, -12, -17, 37, -11, -31, 40, 14, -4, 38, 26, 16, 10, 7, 37, 20, 3, -25, -23, 19, -24, 0, 41, -9, -2, -7, 34, -27, 22, 12, -65, 9, -19, -43, -30, -39, -12, -23, -53, -56, -33, -36, 35, 6, 27, -35, 9, 2, 2, -10, -10, 17, 7, -28, 7, -14, -57, -27, -5, 10, -27, 2, -4, 30, -7, 3, 26, -28, 6, -5, -33, 39, 14, 8, -10, -10, -41, -26, 28, 29, -19, 10, 25, -21, 31, 27, -19, -51, -27, 14, -64, -9, 31, -34, 3, -13, 25, -17, -18, -19, -15, 44, 1, -4, 33, -22, 35, -25, -6, -14, 17, 8, -53, 16, 5, 33, 48, 1, -48, -31, 9, 29, -45, 2, -38, -52, -26, -10, 40, 1, -31, 29, -27, 29, -11, 30, -14, 19, -19, -31, 6, -33, 9, -51, -34, -3, -16, 16, 41, 13, 35, 4, 14, -6, 33, -14, -42, -7, 11, -59, 7, -8, 33, -4, -32, -1, -11, 6, 35, -40, -20, 45, -25, 7, -4, 9, 37, 3, -49, -10, -16, -6, -4, -20, 5, -1, 15, 43, -22, -22, -25, 11, 10, 39, 7, 43, -50, 0, 17, 19, 21, 4, -25, -3, 30, -26, 24, 32, -11, -46, -25, 26, 7, 34, 20, -10, -34, -25, -23, 5, -27, 44, -29, 35, -20, 10, -2, 2, -15, 21, -38, 17, 7, -6, -23, -1, 41, -16, -34, -33, 28, 26, 0, -17, -7, -16, -8, 13, 11, 24, 10, 32, 10, -32, 6, 25, -5, 0, -7, 25, -11, -16, 30, 40, -31, -27, 16, 18, -40, -10, -3, 16, -38, -33, -4, -5, 17, -16, -7, 2, 27, -11, -1, -9, -29, 39, 17, -24, -20, 20, 45, 32, -47, -54, -1, -2, -27, -14, -13, 36, -2, 24, -8, -11, -39, -15, 30, 43, -24, 24, -26, 32, -23, -22, 13, -35, -27, 27, 30, 5, 43, -13, -21, -34, -7, 22, 15, -52, -18, 7, -51, -2, 53, -47, 11, -47, -14, 24, 27, -7, 0, 2, -29, -5, 8, -19, -13, 17, -9, -56, -8, 8, -32, 25, 17, -12, -18, 46, -52, 29, 7, -6, 6, -12, -11, 25, -44, 33, 2, 13, 25, 48, -31, -35, -54, 14, 8, -28, 11, -16, 20, 14, 9, 16, 9, -37, -4, 28, -2, -51, 17, -10, 20, -5, -5, -14, 15, 27, -9, -33, -35, -30, 9, -11, -13, 9, -47, -19, -36, -53, -20, -12, -54, 34, 35, 17, 28, -6, -12, -37, -22, 14, -59, 20, -14, 40, -30, -12, -62, -8, -2, -16, 35, 26, -14, 39, -25, -52, 46, -8, 11, -30, -4, -15, 14, -24, 21, -6, 3, -30, 33, 39, -39, -28, -19, 18, 4, 0, -16, -2, 9, 42, 22, -34, 32, -5, -16, 24, -20, 48, 7, -14, 38, -29, 21, -32, -20, 30, -24, 10, -1, -15, 14, 10, -5, 29, -42, 23, -26, -34, 26, 40, 17, -1, -15, 25, -12, -25, 30, -20, -41, -1, -9, -13, -20, -11, -4, 25, -38, -54, -22, -13, -14, 8, -42, -13, 38, -40, 15, 14, -4, -27, -23, -17, -20, 18, -17, 16, 25, -36, -17, -39, -17, -40, -3, -47, -67, -7, 31, 28, -48, 8, -55, -15, -50, -45, 42, 50, -20, -2, -30, 23, 6, -23, -18, 1, -49, -6, -44, 2, 40, 0, -18, -35, 36, -39, -42, -31, 17, -35, 9, -43, 38, 20, -15, -2, 38, -48, -26, -17, -47, 8, 1, -27, 5, -18, 11, 20, 30, -29, 22, -27, -44, -7, 49, 61, -57, 16, 37, -22, 2, -29, -42, -28, 5, 30, -16, -47, 12, 17, -2, -11, -22, 8, 0, -44, 22, -51, 0, -49, -23, -17, -19, -3, 2, -61, -44, 7, -46, -36, 52, 3, -3, -1, -4, -58, -54, -26, -17, -4, -36, 11, 43, 0, 28, 12, 36, -64, 47, -50, -42, -3, -3, 24, 15, 0, 21, -11, -16, 0, -7, 49, -9, 3, 46, 22, 55, 14, -1, 17, 29, -11, 35, -15, 14, -33, -5, 7, -19, -20, -11, -26, -6, -14, 23, -37, -41, 16, -25, -22, -24, -24, -12, -33, 20, -22, 58, 25, -6, 0, 34, -2, 9, 36, 0, 21, 25, 43, -31, -47, -22, 35, 10, -15, -10, 13, 20, -64, 2, -29, 6, 8, 30, 24, -11, 41, 1, -30, -4, -44, -42, -20, -61, -38, -35, -18, -37, -12, -9, -22, -5, 66, -34, 3, -18, 17, -58, 7, -5, -10, -12, -42, 23, 12, -48, -7, 11, -42, 31, -19, 3, -33, -71, 2, -47, -30, -33, 27, 11, 15, 34, 5, -25, 33, -62, -50, -4, 2, -46, -54, -59, -50, 21, 2, -18, 5, -31, 35, 44, -58, -29, 21, 7, 3, 9, 2, -47, -10, -48, -21, 62, 11, 24, 8, -21, -51, -3, -15, -17, 8, -58, 21, -23, -4, -13, 42, -1, -59, 10, 34, -17, 9, 1, -26, -42, 0, -34, -3, -49, -23, 16, 28, -17, 34, -50, -31, -4, -13, -42, -64, -18, -46, 13, -18, 2, 46, 19, -35, -1, 26, -41, 13, -46, -42, -58, -7, -42, -1, -10, 23, 54, 46, 21, -4, 10, -41, -9, 5, -32, -27, -9, 23, 48, -18, -14, 36, 35, -27, 43, -59, -30, 4, -46, -11, -16, -31, -3, 43, -18, 12, 8, 40, 21, 42, -33, -27, -60, -26, 9, -55, -3, 9, -1, 0, -32, 25, 30, 0, 25, -26, -32, -16, -3, -25, -34, 2, 25, -10, 12, -3, 63, 37, 14, 32, 13, -5, 29, -20, 5, -24, -29, 39, -23, -20, -36, 75, 34, 3, 39, -27, -37, -1, -20, -30, -46, -11, 11, 27, 1, 34}

#define TENSOR_DENSE_KERNEL_0_DEC_BITS {8}

#define TENSOR_DENSE_BIAS_0 {32, 34, 2, -43, -15, -55, -21, -42, -73, 34, -62, -71, 117}

#define TENSOR_DENSE_BIAS_0_DEC_BITS {10}

#define DENSE_BIAS_LSHIFT {0}

#define DENSE_OUTPUT_RSHIFT {8}


/* output q format for each layer */
#define INPUT_1_OUTPUT_DEC 4
#define INPUT_1_OUTPUT_OFFSET 0
#define CONV1D_OUTPUT_DEC 4
#define CONV1D_OUTPUT_OFFSET 0
#define LEAKY_RE_LU_OUTPUT_DEC 4
#define LEAKY_RE_LU_OUTPUT_OFFSET 0
#define CONV1D_1_OUTPUT_DEC 2
#define CONV1D_1_OUTPUT_OFFSET 0
#define LEAKY_RE_LU_1_OUTPUT_DEC 2
#define LEAKY_RE_LU_1_OUTPUT_OFFSET 0
#define FLATTEN_OUTPUT_DEC 2
#define FLATTEN_OUTPUT_OFFSET 0
#define DENSE_OUTPUT_DEC 2
#define DENSE_OUTPUT_OFFSET 0
#define DROPOUT_OUTPUT_DEC 2
#define DROPOUT_OUTPUT_OFFSET 0
#define SOFTMAX_OUTPUT_DEC 7
#define SOFTMAX_OUTPUT_OFFSET 0

/* bias shift and output shift for none-weighted layer */

/* tensors and configurations for each layer */
static int8_t nnom_input_data[450] = {0};

const nnom_shape_data_t tensor_input_1_dim[] = {150, 3};
const nnom_qformat_param_t tensor_input_1_dec[] = {4};
const nnom_qformat_param_t tensor_input_1_offset[] = {0};
const nnom_tensor_t tensor_input_1 = {
    .p_data = (void*)nnom_input_data,
    .dim = (nnom_shape_data_t*)tensor_input_1_dim,
    .q_dec = (nnom_qformat_param_t*)tensor_input_1_dec,
    .q_offset = (nnom_qformat_param_t*)tensor_input_1_offset,
    .qtype = NNOM_QTYPE_PER_TENSOR,
    .num_dim = 2,
    .bitwidth = 8
};

const nnom_io_config_t input_1_config = {
    .super = {.name = "input_1"},
    .tensor = (nnom_tensor_t*)&tensor_input_1
};
const int8_t tensor_conv1d_kernel_0_data[] = TENSOR_CONV1D_KERNEL_0;

const nnom_shape_data_t tensor_conv1d_kernel_0_dim[] = {3, 3, 30};
const nnom_qformat_param_t tensor_conv1d_kernel_0_dec[] = TENSOR_CONV1D_KERNEL_0_DEC_BITS;
const nnom_qformat_param_t tensor_conv1d_kernel_0_offset[] = {0};
const nnom_tensor_t tensor_conv1d_kernel_0 = {
    .p_data = (void*)tensor_conv1d_kernel_0_data,
    .dim = (nnom_shape_data_t*)tensor_conv1d_kernel_0_dim,
    .q_dec = (nnom_qformat_param_t*)tensor_conv1d_kernel_0_dec,
    .q_offset = (nnom_qformat_param_t*)tensor_conv1d_kernel_0_offset,
    .qtype = NNOM_QTYPE_PER_TENSOR,
    .num_dim = 3,
    .bitwidth = 8
};
const int8_t tensor_conv1d_bias_0_data[] = TENSOR_CONV1D_BIAS_0;

const nnom_shape_data_t tensor_conv1d_bias_0_dim[] = {30};
const nnom_qformat_param_t tensor_conv1d_bias_0_dec[] = TENSOR_CONV1D_BIAS_0_DEC_BITS;
const nnom_qformat_param_t tensor_conv1d_bias_0_offset[] = {0};
const nnom_tensor_t tensor_conv1d_bias_0 = {
    .p_data = (void*)tensor_conv1d_bias_0_data,
    .dim = (nnom_shape_data_t*)tensor_conv1d_bias_0_dim,
    .q_dec = (nnom_qformat_param_t*)tensor_conv1d_bias_0_dec,
    .q_offset = (nnom_qformat_param_t*)tensor_conv1d_bias_0_offset,
    .qtype = NNOM_QTYPE_PER_TENSOR,
    .num_dim = 1,
    .bitwidth = 8
};

const nnom_qformat_param_t conv1d_output_shift[] = CONV1D_OUTPUT_RSHIFT;
const nnom_qformat_param_t conv1d_bias_shift[] = CONV1D_BIAS_LSHIFT;
const nnom_conv2d_config_t conv1d_config = {
    .super = {.name = "conv1d"},
    .qtype = NNOM_QTYPE_PER_TENSOR,
    .weight = (nnom_tensor_t*)&tensor_conv1d_kernel_0,
    .bias = (nnom_tensor_t*)&tensor_conv1d_bias_0,
    .output_shift = (nnom_qformat_param_t *)&conv1d_output_shift, 
    .bias_shift = (nnom_qformat_param_t *)&conv1d_bias_shift, 
    .filter_size = 30,
    .kernel_size = {3},
    .stride_size = {3},
    .padding_size = {0, 0},
    .dilation_size = {1},
    .padding_type = PADDING_SAME
};
const int8_t tensor_conv1d_1_kernel_0_data[] = TENSOR_CONV1D_1_KERNEL_0;

const nnom_shape_data_t tensor_conv1d_1_kernel_0_dim[] = {3, 30, 15};
const nnom_qformat_param_t tensor_conv1d_1_kernel_0_dec[] = TENSOR_CONV1D_1_KERNEL_0_DEC_BITS;
const nnom_qformat_param_t tensor_conv1d_1_kernel_0_offset[] = {0};
const nnom_tensor_t tensor_conv1d_1_kernel_0 = {
    .p_data = (void*)tensor_conv1d_1_kernel_0_data,
    .dim = (nnom_shape_data_t*)tensor_conv1d_1_kernel_0_dim,
    .q_dec = (nnom_qformat_param_t*)tensor_conv1d_1_kernel_0_dec,
    .q_offset = (nnom_qformat_param_t*)tensor_conv1d_1_kernel_0_offset,
    .qtype = NNOM_QTYPE_PER_TENSOR,
    .num_dim = 3,
    .bitwidth = 8
};
const int8_t tensor_conv1d_1_bias_0_data[] = TENSOR_CONV1D_1_BIAS_0;

const nnom_shape_data_t tensor_conv1d_1_bias_0_dim[] = {15};
const nnom_qformat_param_t tensor_conv1d_1_bias_0_dec[] = TENSOR_CONV1D_1_BIAS_0_DEC_BITS;
const nnom_qformat_param_t tensor_conv1d_1_bias_0_offset[] = {0};
const nnom_tensor_t tensor_conv1d_1_bias_0 = {
    .p_data = (void*)tensor_conv1d_1_bias_0_data,
    .dim = (nnom_shape_data_t*)tensor_conv1d_1_bias_0_dim,
    .q_dec = (nnom_qformat_param_t*)tensor_conv1d_1_bias_0_dec,
    .q_offset = (nnom_qformat_param_t*)tensor_conv1d_1_bias_0_offset,
    .qtype = NNOM_QTYPE_PER_TENSOR,
    .num_dim = 1,
    .bitwidth = 8
};

const nnom_qformat_param_t conv1d_1_output_shift[] = CONV1D_1_OUTPUT_RSHIFT;
const nnom_qformat_param_t conv1d_1_bias_shift[] = CONV1D_1_BIAS_LSHIFT;
const nnom_conv2d_config_t conv1d_1_config = {
    .super = {.name = "conv1d_1"},
    .qtype = NNOM_QTYPE_PER_TENSOR,
    .weight = (nnom_tensor_t*)&tensor_conv1d_1_kernel_0,
    .bias = (nnom_tensor_t*)&tensor_conv1d_1_bias_0,
    .output_shift = (nnom_qformat_param_t *)&conv1d_1_output_shift, 
    .bias_shift = (nnom_qformat_param_t *)&conv1d_1_bias_shift, 
    .filter_size = 15,
    .kernel_size = {3},
    .stride_size = {3},
    .padding_size = {0, 0},
    .dilation_size = {1},
    .padding_type = PADDING_SAME
};

const nnom_flatten_config_t flatten_config = {
    .super = {.name = "flatten"}
};
const int8_t tensor_dense_kernel_0_data[] = TENSOR_DENSE_KERNEL_0;

const nnom_shape_data_t tensor_dense_kernel_0_dim[] = {255, 13};
const nnom_qformat_param_t tensor_dense_kernel_0_dec[] = TENSOR_DENSE_KERNEL_0_DEC_BITS;
const nnom_qformat_param_t tensor_dense_kernel_0_offset[] = {0};
const nnom_tensor_t tensor_dense_kernel_0 = {
    .p_data = (void*)tensor_dense_kernel_0_data,
    .dim = (nnom_shape_data_t*)tensor_dense_kernel_0_dim,
    .q_dec = (nnom_qformat_param_t*)tensor_dense_kernel_0_dec,
    .q_offset = (nnom_qformat_param_t*)tensor_dense_kernel_0_offset,
    .qtype = NNOM_QTYPE_PER_TENSOR,
    .num_dim = 2,
    .bitwidth = 8
};
const int8_t tensor_dense_bias_0_data[] = TENSOR_DENSE_BIAS_0;

const nnom_shape_data_t tensor_dense_bias_0_dim[] = {13};
const nnom_qformat_param_t tensor_dense_bias_0_dec[] = TENSOR_DENSE_BIAS_0_DEC_BITS;
const nnom_qformat_param_t tensor_dense_bias_0_offset[] = {0};
const nnom_tensor_t tensor_dense_bias_0 = {
    .p_data = (void*)tensor_dense_bias_0_data,
    .dim = (nnom_shape_data_t*)tensor_dense_bias_0_dim,
    .q_dec = (nnom_qformat_param_t*)tensor_dense_bias_0_dec,
    .q_offset = (nnom_qformat_param_t*)tensor_dense_bias_0_offset,
    .qtype = NNOM_QTYPE_PER_TENSOR,
    .num_dim = 1,
    .bitwidth = 8
};

const nnom_qformat_param_t dense_output_shift[] = DENSE_OUTPUT_RSHIFT;
const nnom_qformat_param_t dense_bias_shift[] = DENSE_BIAS_LSHIFT;
const nnom_dense_config_t dense_config = {
    .super = {.name = "dense"},
    .qtype = NNOM_QTYPE_PER_TENSOR,
    .weight = (nnom_tensor_t*)&tensor_dense_kernel_0,
    .bias = (nnom_tensor_t*)&tensor_dense_bias_0,
    .output_shift = (nnom_qformat_param_t *)&dense_output_shift,
    .bias_shift = (nnom_qformat_param_t *)&dense_bias_shift
};

const nnom_softmax_config_t softmax_config = {
    .super = {.name = "softmax"}
};
static int8_t nnom_output_data[13] = {0};

const nnom_shape_data_t tensor_output0_dim[] = {13};
const nnom_qformat_param_t tensor_output0_dec[] = {SOFTMAX_OUTPUT_DEC};
const nnom_qformat_param_t tensor_output0_offset[] = {0};
const nnom_tensor_t tensor_output0 = {
    .p_data = (void*)nnom_output_data,
    .dim = (nnom_shape_data_t*)tensor_output0_dim,
    .q_dec = (nnom_qformat_param_t*)tensor_output0_dec,
    .q_offset = (nnom_qformat_param_t*)tensor_output0_offset,
    .qtype = NNOM_QTYPE_PER_TENSOR,
    .num_dim = 1,
    .bitwidth = 8
};

const nnom_io_config_t output0_config = {
    .super = {.name = "output0"},
    .tensor = (nnom_tensor_t*)&tensor_output0
};
/* model version */
#define NNOM_MODEL_VERSION (10000*0 + 100*4 + 3)

/* nnom model */
static nnom_model_t* nnom_model_create(void)
{
	static nnom_model_t model;
	nnom_layer_t* layer[9];

	check_model_version(NNOM_MODEL_VERSION);
	new_model(&model);

	layer[0] = input_s(&input_1_config);
	layer[1] = model.hook(conv2d_s(&conv1d_config), layer[0]);
	layer[2] = model.active(act_leaky_relu(0.300000f), layer[1]);
	layer[3] = model.hook(conv2d_s(&conv1d_1_config), layer[2]);
	layer[4] = model.active(act_leaky_relu(0.300000f), layer[3]);
	layer[5] = model.hook(flatten_s(&flatten_config), layer[4]);
	layer[6] = model.hook(dense_s(&dense_config), layer[5]);
	layer[7] = model.hook(softmax_s(&softmax_config), layer[6]);
	layer[8] = model.hook(output_s(&output0_config), layer[7]);
	model_compile(&model, layer[0], layer[8]);
	return &model;
}
