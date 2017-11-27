#ifndef UPLOAD_GPU
#define UPLOAD_GPU

//void cudaCopy(void ** dst, void ** src, int bytes);
void cudaCopy(unsigned char* dst, unsigned char * src, int bytes);
void cuda2HostArray(cudaArray* c_array, unsigned char* h_array, int width, int height, int bytes);
void cudaMemCpyArr(cudaArray* c_array, unsigned char* h_array, int width, int height, int bytes);
//void cudaMemCpyArr(const cudaArray_t * c_array, unsigned char* h_array, int width, int height, int bytes)
// void cudaMemCpyArr(const struct cudaArray * c_array, unsigned char* h_array, int width, int height, int bytes);
#endif
