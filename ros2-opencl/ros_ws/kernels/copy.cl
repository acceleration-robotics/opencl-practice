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


__kernel void copy2(__read_only image2d_t in,
                   __write_only image2d_t out)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int2 pos = (int2)(x, y);
    uint4 pixel = read_imageui(in, smp, pos);
    
    write_imageui(out, pos, pixel);
}

__kernel void copyImageBuffer(__global const uchar* input, __global uchar* output) {
            int gid_x = get_global_id(0);
            int gid_y = get_global_id(1);
            
            
            if (gid_x < get_global_size(0) && gid_y < get_global_size(1)) {
                int pos = gid_y * get_global_size(0) + gid_x;
                uchar4 pixel = vload4(pos, input);
                vstore4(pixel, pos, output);
            }
        }

__kernel void copyImageBuffer2(__global const uchar* input, __global uchar* output) {
            int gid_x = get_global_id(0);
            int gid_y = get_global_id(1);
            
            
            if (gid_x < get_global_size(0) && gid_y < get_global_size(1)) {
                int pos = gid_y * get_global_size(0) + gid_x;
                uchar4 pixel = vload4(pos, input);
                vstore4(pixel, pos, output);
            }
        }

__kernel void resizeImageBuffer(__global const uchar* input, __global uchar* output,
                                __const int inWidth, __const int scale) 
{
    const int gx = get_global_id(0);
    const int gy = get_global_id(1);
    const int2 pos = {gx, gy};

    const int outWidth = scale * inWidth;
    
    float2 srcpos = (float2)((gx + 0.4955f) / scale, (gy + 0.4955f) / scale);
    // float2 srcpos = (float2)((g))
    int2 ipos = convert_int2(srcpos);
    int ipos_1d = ipos.y * inWidth + ipos.x;
    int opos_1d = pos.y * outWidth + pos.x;
    // output[opos_1d] = input[ipos_1d];

    uchar4 pixel = vload4(ipos_1d, input);
    // for (int i = 0; i < 2; i++) {
    //     for (int j = 0; j < 2; j++) {
    //        int opos_1d = (ipos.y + j) * inWidth + (ipos.x + i);
    //        vstore4(pixel, opos_1d, output);
    //    }
    // }
    vstore4(pixel, opos_1d, output);

}

__kernel void rectifyImageBuffer(__global const uchar* inputImage,
                          __global uchar* outputImage,
                          __global float* M)
{
    float fx = M[0];
    float fy = M[1];
    float cx = M[2];
    float cy = M[3];
    float k1 = M[4];
    float k2 = M[5];
    float k3 = M[6];
    float p1 = M[7];
    float p2 = M[8];
    float r11 = M[9];
    float r12 = M[10];
    float r13 = M[11];
    float r21 = M[12];
    float r22 = M[13];
    float r23 = M[14];
    float r31 = M[15];
    float r32 = M[16];
    float r33 = M[17];
    float p11 = M[18];
    float p12 = M[19];
    float p13 = M[20];
    float p14 = M[21];
    float p21 = M[22];
    float p22 = M[23];
    float p23 = M[24];
    float p24 = M[25];
    float p31 = M[26];
    float p32 = M[27];
    float p33 = M[28];
    float p34 = M[29];

    int x = get_global_id(0);
    int y = get_global_id(1);
    int2 pos = (int2)(x, y);

    // Calculate pixel coordinates in rectified space
    float2 rect_pos = (float2)((pos.x - cx) / fy, (pos.y - cy) / fy);

    // Perform distortion correction
    float r2 = rect_pos.x * rect_pos.x + rect_pos.y * rect_pos.y;
    float r4 = r2 * r2;
    float r6 = r2 * r4;    
    float radialDistortion = 1.0 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2;
    float tangentialDistortionX = 2.0 * p1 * rect_pos.x * rect_pos.y + p2 * (r2 + 2.0 * rect_pos.x * rect_pos.x);
    float tangentialDistortionY = p1 * (r2 + 2.0 * rect_pos.y*rect_pos.y) + 2.0 * p2 * rect_pos.x * rect_pos.y;

    // Apply distortion correction
    float2 dist_pos = (float2)((rect_pos.x * radialDistortion + tangentialDistortionX), 
                                (rect_pos.y * radialDistortion + tangentialDistortionY));

    // Apply rectification matrix 
    float rrect_x = r11 * dist_pos.x + r12 * dist_pos.y + r13;
    float rrect_y = r21 * dist_pos.y + r22 * dist_pos.y + r23;

    // Map rectified coordinates back to image space
    // int2 ipos = (int2)((dist_pos.x * fx + cx + 0.5f), (dist_pos.y * fy + cy + 0.5f));
    // int2 ipos = (int2)((rect_pos.x * fx + cx + 0.5f), (rect_pos.y * fy + cy + 0.5f));
    int2 ipos = (int2)((rrect_x * fx + cx + 0.5f), (rrect_y * fy + cy + 0.5f));
    int ipos_1d = ipos.y * get_global_size(0) + ipos.x;
    int pos_1d = pos.y * get_global_size(0) + pos.x;
    uchar4 pixel = vload4(ipos_1d, inputImage);
    vstore4(pixel, pos_1d, outputImage);
}