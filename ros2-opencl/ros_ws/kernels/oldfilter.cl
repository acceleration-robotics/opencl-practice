int2 coord = (int2)(get_global_id(0), get_global_id(1));
    int halfWindow = windowSize/2;
    uint4 pixel_u;
    float4 pixel_f;
    // Loop over each element of the gaussian kernel and add the value
    float4 sum_filter = (float4)(0, 0, 0, 0);
    uint offset = windowSize/2;
    int i, j, ifilter, jfilter;
    for (i=-offset, ifilter=0; i<=offset; i++, ifilter++)
    {
        for (j=-offset, jfilter=0; j<offset; j++, jfilter++)
        {
            int2 pixelpos = (int2)(i, j);
            pixel_u = read_imageui(iimage, image_sampler, pixelpos);
            pixel_f = convert_float_rte(pixel_u);
            sum_filter += pixel_f*filter[ifilter*windowSize+jfilter];
        }
    }

    // Assign alpha from original image pixel's alpha
    pixel_u = read_imageui(iimage, image_sampler, coord);
    uint4 op_pixel = convert_uint4_sat_rte(sum_filter);
    // op_pixel.w = pixel_u.w;
    write_imageui(oimage, coord, op_pixel);


    // for (uint i=0; i < windowSize; ++i)
    // {
    //    for (uint j = 0; j < windowSize; ++j)
    //    {
    //        int2 pixelpos = (int2)(coord.x - offset + i, coord.y - offset + j);
    //        pixel_u = read_imageui(iimage, image_sampler, pixelpos);
    //        pixel_f = convert_float_rte(pixel_u);
    //        sum_filter += pixel_f*filter[i*windowSize+j];
    //    }
    // }

    // Assign alpha from original image pixel's alpha
    // pixel_u = read_imageui(iimage, image_sampler, coord);
    // uint4 op_pixel = convert_uint4_sat_rte(sum_filter);
    // op_pixel.w = pixel_u.w;
    // write_imageui(oimage, coord, op_pixel);


-----------------
__constant sampler_t image_sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

__kernel void filter_kernel(__read_only image2d_t iimage,
                            __write_only image2d_t oimage, 
                            __constant float *filter,
                            int windowSize)
{

    int x = get_global_id(0);
    int y = get_global_id(1);
    uint4 pixelValue;
    int halfWindow = windowSize/2;
    float4 computedFilter= (float4)(0.0f, 0.0f, 0.0f, 0.0f);
    int i, j, ifilter, jfilter;
    for(i=-halfWindow, ifilter=0; i<=halfWindow; i++, ifilter++)
    {
        for(j=-halfWindow, jfilter=0; j<=halfWindow; j++,jfilter++)
        {
            pixelValue = read_imageui(iimage, image_sampler,(int2)(x+i, y+j));
            float4 pixel_f = convert_float4(pixelValue);
            computedFilter += filter[ifilter*windowSize+jfilter]*pixel_f;
        }
    }
    uint4 out_pixel = convert_uint4_sat_rte(computedFilter);
    write_imageui(oimage, (int2)(x, y), out_pixel);


    
}
--------------------------
__constant sampler_t image_sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

__kernel void filter_kernel(__read_only image2d_t iimage,
                            __write_only image2d_t oimage, 
                            __constant float *filter,
                            int windowSize)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int halfWindow = windowSize/2;
    uint4 pixel_u;
    float4 pixel_f;
    float4 computedFilter= (float4)(0.0f, 0.0f, 0.0f, 0.0f);
    int i, j, ifilter, jfilter;
    for(i=-halfWindow, ifilter=0; i<=halfWindow; i++, ifilter++)
    {
        for(j=-halfWindow, jfilter=0; j<=halfWindow; j++,jfilter++)
        {
            pixel_u = read_imageui(iimage, image_sampler,(int2)(x+i, y+j));
            pixel_f = convert_float4(pixel_u);
            computedFilter += filter[ifilter*windowSize+jfilter]*pixel_f;
        }
    }
    uint4 op_pixel = convert_uint4(computedFilter);
    write_imageui(oimage, (int2)(x, y), op_pixel);
}

-----------------------------
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
    write_imagef(oimage, (int2)(x, y), computedFilter);
    // write_imageui(oimage, (int2)(x, y), out_pixel);
}


---------------
__constant sampler_t image_sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

__kernel void filter_kernel(__read_only image2d_t iimage,
                            __write_only image2d_t oimage, 
                            __constant float *filter,
                            int windowSize)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int halfWindow = windowSize/2;
    float4 computedFilter= (float4)(0, 0, 0, 0);
    int i, j, ifilter, jfilter;
    for(i=-halfWindow, ifilter=0; i<halfWindow; i++, ifilter++)
    {
        for(j=-halfWindow, jfilter=0; j<halfWindow; j++,jfilter++)
        {
            uint4 pixel_u = (uint4) read_imageui(iimage, image_sampler,(int2)(x+i, y+j));
            float4 pixel_f = convert_float4(pixel_u);
            computedFilter += filter[ifilter*windowSize+jfilter]*pixel_f;
            printf("computedFilter = %2.2v4hlf\n", (float4) computedFilter);
            // printf("inpixel = %2.2v4hlf\n", pixel_f);
            // printf("in pixel = (%#v4hx)\n", pixel_u);
        }
    }
    float4 inpixel = read_imagef(iimage, image_sampler, (int2)(x,y));
    // printf("in pixel = (%#v4hx)\n", inpixel);
    // printf("computedFilter = %2.2v4hlf\n", computedFilter);
    
    float4 out_pixel = computedFilter;
    out_pixel.w = inpixel.w;
    // uint4 out_pixel = (uint4) convert_uint4(computedFilter);
    // uint4 out_pixel = (uint4) convert_uint4_sat(computedFilter);
    // printf("outpixel uint = (%#v4hx)\n", out_pixel);
    write_imagef(oimage, (int2)(x, y), computedFilter);
    // write_imageui(oimage, (int2)(x, y), out_pixel);
}