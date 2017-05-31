
#include "convolution_fftw.h"



// Code adapted from gsl/fft/factorize.c
void FFTW_Convolution::factorize (int n,
                                  int *n_factors,
                                  int factors[],
                                  int * implemented_factors)
{
    int nf = 0;
    int ntest = n;
    int factor;
    int i = 0;

    if (n == 0)
    {
        printf("Length n must be positive integer\n");
        return ;
    }

    if (n == 1)
    {
        factors[0] = 1;
        *n_factors = 1;
        return ;
    }

    /* deal with the implemented factors */

    while (implemented_factors[i] && ntest != 1)
    {
        factor = implemented_factors[i];
        while ((ntest % factor) == 0)
        {
            ntest = ntest / factor;
            factors[nf] = factor;
            nf++;
        }
        i++;
    }

    // Ok that's it
    if(ntest != 1)
    {
        factors[nf] = ntest;
        nf++;
    }

    /* check that the factorization is correct */
    {
        int product = 1;

        for (i = 0; i < nf; i++)
        {
            product *= factors[i];
        }

        if (product != n)
        {
            printf("factorization failed");
        }
    }

    *n_factors = nf;
}

int FFTW_FACTORS[7] = {13,11,7,5,3,2,0}; // end with zero to detect the end of the array

bool FFTW_Convolution::is_optimal(int n, int * implemented_factors)
{
    // We check that n is not a multiple of 4*4*4*2
    if(n % 4*4*4*2 == 0)
        return false;

    int nf=0;
    int factors[64];
    int i = 0;
    factorize(n, &nf, factors,implemented_factors);

    // We just have to check if the last factor belongs to GSL_FACTORS
    while(implemented_factors[i])
    {
        if(factors[nf-1] == implemented_factors[i])
            return true;
        ++i;
    }
    return false;
}

int FFTW_Convolution::find_closest_factor(int n, int * implemented_factor)
{
    int j;
    if(is_optimal(n,implemented_factor))
        return n;
    else
    {
        j = n+1;
        while(!is_optimal(j,implemented_factor))
            ++j;
        return j;
    }
}

void FFTW_Convolution::init_workspace(Workspace & ws, Convolution_Mode mode, int h_src, int w_src, int h_kernel, int w_kernel)
{
    ws.h_src = h_src;
    ws.w_src = w_src;
    ws.h_kernel = h_kernel;
    ws.w_kernel = w_kernel;
    ws.mode = mode;

    switch(mode)
    {
    case LINEAR:
        // Same Linear convolution
        ws.h_fftw = find_closest_factor(h_src + int(h_kernel/2.0),FFTW_FACTORS);
        ws.w_fftw = find_closest_factor(w_src + int(w_kernel/2.0),FFTW_FACTORS);
        ws.h_dst = h_src;
        ws.w_dst = w_src;
        break;
    case CIRCULAR:
        ws.h_dst = h_src;
        ws.w_dst = w_src;
        ws.h_fftw = ws.h_dst;
        ws.w_fftw = ws.w_dst;
        break;
    default:
        printf("Unrecognized convolution mode, possible modes are :\n");
        printf("   - LINEAR \n");
        printf("   - CIRCULAR\n");
    }

    ws.in_src = new double[ws.h_fftw * ws.w_fftw];
    ws.out_src = (double*) fftw_malloc(sizeof(fftw_complex) * ws.h_fftw * (ws.w_fftw/2+1));
    ws.in_kernel = new double[ws.h_fftw * ws.w_fftw];
    ws.out_kernel = (double*) fftw_malloc(sizeof(fftw_complex) * ws.h_fftw * (ws.w_fftw/2+1));
    ws.dst_fft = new double[ws.h_fftw * ws.w_fftw];
    ws.dst = new double[ws.h_dst * ws.w_dst];

    // Initialization of the plans
    ws.p_forw_src = fftw_plan_dft_r2c_2d(ws.h_fftw, ws.w_fftw, ws.in_src, (fftw_complex*)ws.out_src, FFTW_ESTIMATE);
    ws.p_forw_kernel = fftw_plan_dft_r2c_2d(ws.h_fftw, ws.w_fftw, ws.in_kernel, (fftw_complex*)ws.out_kernel, FFTW_ESTIMATE);

    // The backward FFT takes ws.out_kernel as input !!
    ws.p_back = fftw_plan_dft_c2r_2d(ws.h_fftw, ws.w_fftw, (fftw_complex*)ws.out_kernel, ws.dst_fft, FFTW_ESTIMATE);
}

void FFTW_Convolution::clear_workspace(Workspace & ws)
{
    delete[] ws.in_src;
    fftw_free((fftw_complex*)ws.out_src);
    delete[] ws.in_kernel;
    fftw_free((fftw_complex*)ws.out_kernel);

    delete[] ws.dst_fft;
    delete[] ws.dst;

    // Destroy the plans
    fftw_destroy_plan(ws.p_forw_src);
    fftw_destroy_plan(ws.p_forw_kernel);
    fftw_destroy_plan(ws.p_back);
}

// Compute the circular convolution of src and kernel modulo ws.h_fftw, ws.w_fftw
// using the Fast Fourier Transform
// The result is in ws.dst
void FFTW_Convolution::fftw_circular_convolution(Workspace &ws, double * src, double * kernel)
{
    double * ptr, *ptr_end, *ptr2;

    // Reset the content of ws.in_src
    for(ptr = ws.in_src, ptr_end = ws.in_src + ws.h_fftw*ws.w_fftw ; ptr != ptr_end ; ++ptr)
        *ptr = 0.0;
    for(ptr = ws.in_kernel, ptr_end = ws.in_kernel + ws.h_fftw*ws.w_fftw ; ptr != ptr_end ; ++ptr)
        *ptr = 0.0;

    // Then we build our periodic signals
    //ptr = src;
    for(int i = 0 ; i < ws.h_src ; ++i)
        for(int j = 0 ; j < ws.w_src ; ++j, ++ptr)
            ws.in_src[(i%ws.h_fftw)*ws.w_fftw+(j%ws.w_fftw)] += src[i*ws.w_src + j];
    //ptr = kernel;
    for(int i = 0 ; i < ws.h_kernel ; ++i)
        for(int j = 0 ; j < ws.w_kernel ; ++j, ++ptr)
            ws.in_kernel[(i%ws.h_fftw)*ws.w_fftw+(j%ws.w_fftw)] += kernel[i*ws.w_kernel + j];

    // And we compute their packed FFT
    fftw_execute(ws.p_forw_src);
    fftw_execute(ws.p_forw_kernel);

    // Compute the element-wise product on the packed terms
    // Let's put the element wise products in ws.in_kernel
    double re_s, im_s, re_k, im_k;
    for(ptr = ws.out_src, ptr2 = ws.out_kernel, ptr_end = ws.out_src+2*ws.h_fftw * (ws.w_fftw/2+1); ptr != ptr_end ; ++ptr, ++ptr2)
    {
        re_s = *ptr;
        im_s = *(++ptr);
        re_k = *ptr2;
        im_k = *(++ptr2);
        *(ptr2-1) = re_s * re_k - im_s * im_k;
        *ptr2 = re_s * im_k + im_s * re_k;
    }

    // Compute the backward FFT
    // Carefull, The backward FFT does not preserve the output
    fftw_execute(ws.p_back);
    // Scale the transform
    for(ptr = ws.dst_fft, ptr_end = ws.dst_fft + ws.w_fftw*ws.h_fftw ; ptr != ptr_end ; ++ptr)
        *ptr /= double(ws.h_fftw*ws.w_fftw);

    // That's it !
}

void FFTW_Convolution::convolve(Workspace &ws, double * src,double * kernel)
{
    if(ws.h_fftw <= 0 || ws.w_fftw <= 0)
        return;

    // Compute the circular convolution
    fftw_circular_convolution(ws, src, kernel);

    // Depending on the type of convolution one is looking for, we extract the appropriate part of the result from out_src
    int h_offset, w_offset;

    switch(ws.mode)
    {
    case LINEAR:
        // Same linear convolution
        // Here we just keep the first [h_filt/2:h_filt/2+h_dst-1 ; w_filt/2:w_filt/2+w_dst-1] real part elements of out_src
        h_offset = int(ws.h_kernel/2.0);
        w_offset = int(ws.w_kernel/2.0);
        for(int i = 0 ; i < ws.h_dst ; ++i)
            memcpy(&ws.dst[i*ws.w_dst], &ws.dst_fft[(i+h_offset)*ws.w_fftw+w_offset], ws.w_dst*sizeof(double));
        break;
    case CIRCULAR:
        // Circular convolution
        // We copy the first [0:h_dst-1 ; 0:w_dst-1] real part elements of out_src
        for(int i = 0 ; i < ws.h_dst ; ++i)
            for(int j = 0 ; j < ws.w_dst ; ++j)
                ws.dst[i*ws.w_dst + j] = ws.dst_fft[((i+int(ws.h_kernel/2.0))%ws.h_fftw)*ws.w_fftw + (j + int(ws.w_kernel/2.0))%ws.w_fftw];
        break;
    default:
        printf("Unrecognized convolution mode, possible modes are :\n");
        printf("   - LINEAR \n");
        printf("   - CIRCULAR \n");
    }
}
