// to upload the code from GPu to CPU

#include <stdint.h>
#include <iostream>
//headers for image msg
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/image_encodings.h>

//set static global var texture. texture cannot be passed as argument !! check if param is correct
texture<float, cudaTextureType2D, cudaReadModeElementType> textureRef;


	__global__
void copyKernel(unsigned char* d_array, int width, int height, float theta)
{
	// Calculate normalized texture coordinates !! check if correct/ what it means
	unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;

	unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

	float u = x / (float)width;

	float v = y / (float)height;

	// Transform coordinates
	u -= 0.5f;

	v -= 0.5f; 

	float tu = u * cosf(theta) - v * sinf(theta) + 0.5f ;

	float tv = v * cosf(theta) + u * sinf(theta) + 0.5f ;

	// Read from texture and write to global memory
	d_array[y * width + x] = tex2D(textureRef, tu, tv);
//	d_array[y * width + x] = 100;
	return;
}

__global__ void fakeArray(unsigned char* d_array, int width, int height, float theta)
{
	for(int i = 0 ; i <width; ++i)
		for(int j = 0 ; j < height; ++j)
		{
			d_array[i*width + j] = 10;
		}
	return;
}


void cuda2sharedMem(unsigned char* d_array, cudaArray_t c_array, int width, int height)
{
	//cudaSetDevice(1);  // to use shared memory space

	// ---- convert to texture ---- //

	//set texture param !! check to see what does these param mean
	textureRef.addressMode[0] = cudaAddressModeWrap;
	textureRef.addressMode[1] = cudaAddressModeWrap;
	textureRef.filterMode = cudaFilterModeLinear;
	textureRef.normalized = true;

	//set cuda array channel descp !! look into how to get this from the cuda array
	cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindUnsigned);

	cudaBindTextureToArray(textureRef, c_array, channelDesc); 


	// ---- copy to an array on the GPU ---- //

	// unsigned char* d_array;
	// int size = width * height * sizeof(uint8_t);
	// cudaMalloc(&d_array, size);

	dim3 blockSize(16, 16);
	dim3 numBlocks( (width + blockSize.x - 1) / blockSize.x, (height + blockSize.y - 1) / blockSize.y );

	// std::cerr << " copying with " << numBlocks.x + numBlocks.y << " of blocks, of size " << blockSize.x  << " by " << blockSize.y << "each\n";
	// std::cerr << "copying the image of width: " << width;
	//call kernel to copy
	copyKernel<<<numBlocks,blockSize>>>(d_array, width, height, 90.0);

//	fakeArray<<<1,1>>>(d_array, width, height, 90.0);
//	std::cerr << "cuda generted fake img\n"; 
	cudaDeviceSynchronize();

	// ---- unbind n release resources ---- //
	//unbind to texture? !! ?
	cudaUnbindTexture(textureRef);

	return;
}


void cudaCopy(unsigned char* dst, unsigned char* src, int bytes)
{
	cudaMemcpy(dst, src, bytes, cudaMemcpyDeviceToHost);
	//	cudaMemcpy(dst, src, bytes, cudaMemcpyHostToHost);
	return;
}


void cuda2HostArray(cudaArray_t c_array, unsigned char* h_array, int width, int height, int bytes)
{
	unsigned char* d_array;
	int size = bytes; //width * height * sizeof(uint8_t);
	cudaError_t code;
	code = cudaMalloc(&d_array, size);
	if(code != cudaSuccess)
	{
		std::cerr << "cuda malloc failed\n";
	}

	// d_array = cuda2sharedMem(d_array, c_array, width, height);
	cuda2sharedMem(d_array, c_array, width, height);

	// cudaMemcpy(h_array, * c_array, bytes, cudaMemcpyDeviceToHost);
	// cudaMemcpy(h_array, d_array, bytes, cudaMemcpyHostToHost);
	cudaMemcpy(h_array, d_array, bytes, cudaMemcpyDeviceToHost);

	cudaFree(d_array);

	std::cerr << "returned resources, leaving cuda\n";

	return;
}


void cudaMemCpyArr(cudaArray_t  c_array, unsigned char* h_array, int width, int height, int bytes)
// void cudaMemCpyArr(const struct cudaArray * c_array, unsigned char* h_array, int width, int height, int bytes)
{
/**
cudaError_t cudaMemcpy2DFromArray 	( 	void *  	dst,
		size_t  	dpitch,
		const struct cudaArray *  	src,
		size_t  	wOffset,
		size_t  	hOffset,
		size_t  	width,
		size_t  	height,
		enum cudaMemcpyKind  	kind	 
	) 	
**/	
/**	cudaError_t code; 	
	code = cudaMemcpy2DFromArray(h_array, width, c_array, 0, 0, width, height, cudaMemcpyDeviceToHost);
	if(code != cudaSuccess)
	{
		std::cerr << "cuda memcpy array failed: " << cudaGetErrorString(code) << std::endl;
	}
**/

/**	cudaError_t cudaMemcpyFromArray 	( 	void *  	dst,
		const struct cudaArray *  	src,
		size_t  	wOffset,
		size_t  	hOffset,
		size_t  	count,
		enum cudaMemcpyKind  	kind	 
	) 	
**/

	float *testArray = new float[width * height * 4];	
	// uint8_t *testArray = new uint8_t[width * height * 4];	
	cudaArray* cuArray;
	cudaChannelFormatDesc channelD = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);
	cudaMallocArray(&cuArray, &channelD, width*height*4, 0); 
	cudaError_t code;       
        // code = cudaMemcpyFromArray(h_array, c_array, 0, 0, bytes, cudaMemcpyDeviceToHost);
        code = cudaMemcpyFromArray(testArray, cuArray, 0, 0, 1, cudaMemcpyDeviceToHost);
        if(code != cudaSuccess)
        {
                std::cerr << "cuda memcpy array failed: " << cudaGetErrorString(code) << std::endl;
        }

	cudaDeviceSynchronize();

	return;
}
