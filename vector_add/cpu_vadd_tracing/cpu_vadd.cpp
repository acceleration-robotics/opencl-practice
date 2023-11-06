#include <iostream>
#include "cpu_vadd_tp.h"

void vector_add(const int * A, const int * B, const int size, int &counter, int * C) {
    for (int i=0; i < size; i++) {
        C[i] = A[i] + B[i];
        std::cout << C[i] << "\n";
    }
    counter++;
    // lttng_ust_tracepoint(vadd, cpu_tp, counter, "End of vadd");
}

int main() {
    int callcounter = 0;
    int v_size;
    std::cout << "Enter number of elements in vector : " << "\n";
    std::cin >> v_size;
    int A_h[v_size], B_h[v_size], C_h[v_size];
    for (int i=0; i<v_size; i++) {
        A_h[i] = rand() % 100;
        B_h[i] = rand() % 100;
    }
    lttng_ust_tracepoint(vadd, cpu_tp, v_size, "cpu_vadd start");
    vector_add(A_h, B_h, v_size, callcounter, C_h);
    lttng_ust_tracepoint(vadd, cpu_tp, v_size, "cpu_vadd end");
    return 0;
}