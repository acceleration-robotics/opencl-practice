const sampler_t smp = CLK_NORMALIZED_COORDS_TRUE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;
__kernel void resize(__read_only image2d_t in,
                   __write_only image2d_t out,
                   int scaling_factor)
{
    const int gx = get_global_id(0);
    const int gy = get_global_id(1);
    const int2 pos = { gx, gy };

    if (pos.x >= get_image_width(out) || pos.y >= get_image_height(out))
        return;

    float2 srcpos = {(pos.x + 0.4995f) / scaling_factor, (pos.y + 0.4995f) / scaling_factor};
    int2 SrcSize = (int2)(get_image_width(in), get_image_height(in));

    uint4 pixel;

    int2 ipos = convert_int2(srcpos);
    if (ipos.x < 0 || ipos.x >= get_image_width(in) || ipos.y < 0 || ipos.y >= get_image_height(in))
        pixel = 0;
    else
        pixel = read_imageui(in, smp, ipos);

    write_imageui(out, pos, pixel);
}