// memcpy from gpu to cpu

void cudaCopy(unsigned char* dst, unsigned char* src, int bytes)
{
	cudaMemcpy(dst, src, bytes, cudaMemcpyDeviceToHost);
	return;
}
