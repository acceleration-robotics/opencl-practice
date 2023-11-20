const sampler_t smp = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;
__kernel void resize(__read_only image2d_t in,
                   __write_only image2d_t out,
                   __const int scaling_factor)
{
    uint4 pixel;
    float2 inpos = (float2)(get_global_id(0) + 0.4995f/scaling_factor,
                                  get_global_id(1) + 0.4995f/scaling_factor);
    int2 outpos = (int2)(scaling_factor*get_global_id(0), scaling_factor*get_global_id(1));

    for(int i=0; i<scaling_factor; i++) {
        for(int j=0; j<scaling_factor; j++) {
            pixel = read_imageui(in, smp, (float2)(inpos + (float2)(i/scaling_factor, i/scaling_factor)));
            write_imageui(out, outpos + (int2)(i, j), pixel);
        }
    }
    
}