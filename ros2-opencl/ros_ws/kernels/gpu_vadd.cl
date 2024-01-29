__kernel void vector_add(global const int * A, global int * B) {
    B[get_global_id(0)] = 2*A[get_global_id(0)] ;
    printf("%d \n", B[get_global_id(0)]);
}

__kernel void vector_sub(global const int * A, global int * B) {
    B[get_global_id(0)] = 0.5*A[get_global_id(0)];
    printf("%d \n", B[get_global_id(0)]);
}

__kernel void copy(global const uint * A, global uint * B) {
    B[get_global_id(0)] = A[get_global_id(0)];
}

