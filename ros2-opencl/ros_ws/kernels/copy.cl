const sampler_t smp = CLK_NORMALIZED_COORDS_TRUE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;
__kernel void copy(__read_only image2d_t in,
                   __write_only image2d_t out)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int2 pos = (int2)(x, y);
    uint4 pixel = read_imageui(in, smp, pos);
    
    write_imageui(out, pos, pixel);
}

__kernel void copyBuffer(__global const uint4* inBuffer,
                   __global uint4* outBuffer)
{
    int x = get_global_id(0);
    outBuffer[x] = inBuffer[x];
    printf("%d \n", inBuffer[x]);
}