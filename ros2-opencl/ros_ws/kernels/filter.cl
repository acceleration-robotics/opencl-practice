__constant sampler_t image_sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

__kernel void filter_kernel(__read_only image2d_t iimage,
                            __write_only image2d_t oimage, 
                            __constant float *filter,
                            int windowSize)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int halfWindow = windowSize/2;
    uint4 pixelValue;
    float4 computedFilter= (float4)(0, 0, 0, 0);
    int i, j, ifilter, jfilter;
    for(i=-halfWindow, ifilter=0; i<=halfWindow; i++, ifilter++)
    {
        for(j=-halfWindow, jfilter=0; j<=halfWindow; j++,jfilter++)
        {
            pixelValue = read_imageui(iimage, image_sampler,(int2)(x+i, y+j));
            float4 pixel_f = (float4) convert_float4(pixelValue);
            computedFilter += filter[jfilter*windowSize+ifilter]*pixel_f;
        }
    }
    uint4 inpixel = read_imageui(iimage, image_sampler, (int2)(x,y));
    // float4 out_pixel = computedFilter;
    // out_pixel.w = inpixel.w;
    // uint4 out_pixel = (uint4) convert_uint4(computedFilter);
    uint4 out_pixel = (uint4) convert_uint4_sat(computedFilter);
    // write_imagef(oimage, (int2)(x, y), computedFilter);
    write_imageui(oimage, (int2)(x, y), out_pixel);
}
