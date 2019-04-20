//
// Created by yyhu on 4/19/19.
//

#ifndef STEREOUTILS_ARRAYOUTPUT_HPP
#define STEREOUTILS_ARRAYOUTPUT_HPP

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

template <typename T>
void write_array(const std::string& fn, const T* array, const size_t n, const std::string& d="\n")
{
    if ( NULL == array )
    {
        std::stringstream ss;
        ss << "array is NULL.";
        throw std::runtime_error(ss.str());
    }

    // Open the file.
    std::ofstream ofs(fn);

    if ( !ofs.good() )
    {
        std::stringstream ss;
        ss << "ofs is not good.";
        throw std::runtime_error(ss.str());
    }

    ofs << n << std::endl;

    for ( size_t i = 0; i < n; ++i )
    {
        ofs << array[i] << d;
    }

    ofs.close();
}

template <typename T>
void write_array_2D(const std::string& fn, const T* array, const size_t rows, const size_t cols,
    const int w=0, const std::string& rd="\n", const std::string& cd=", ")
{
    if ( NULL == array )
    {
        std::stringstream ss;
        ss << "array is NULL.";
        throw std::runtime_error(ss.str());
    }

    // Open the file.
    std::ofstream ofs(fn);

    if ( !ofs.good() )
    {
        std::stringstream ss;
        ss << "ofs is not good.";
        throw std::runtime_error(ss.str());
    }

    ofs << rows << ", " << cols << std::endl;

    if ( w <= 0 )
    {
        size_t rowStart;

        for ( size_t i = 0; i < rows; ++i )
        {
            size_t j;

            rowStart = i*cols;

            for ( j = 0; j < cols - 1; ++j )
            {
                ofs << array[rowStart + j] << cd;
            }

            ofs << array[rowStart+ j] << rd;
        }
    }
    else
    {
        size_t rowStart;

        for ( size_t i = 0; i < rows; ++i )
        {
            size_t j;

            rowStart = i*cols;

            for ( j = 0; j < cols - 1; ++j )
            {
                ofs << std::setw(w) << array[rowStart + j] << cd;
            }

            ofs << std::setw(w) << array[rowStart+ j] << rd;
        }
    }


    ofs.close();
}

template <typename T>
void write_array_3D(const std::string& fn, const T* array,
        const size_t s2, const size_t s1, const size_t s0,
        const int w=0,
        const std::string& d2="#", const std::string& d1="\n", const std::string& d0=", ")
{
    if ( NULL == array )
    {
        std::stringstream ss;
        ss << "array is NULL.";
        throw std::runtime_error(ss.str());
    }

    // Open the file.
    std::ofstream ofs(fn);

    if ( !ofs.good() )
    {
        std::stringstream ss;
        ss << "ofs is not good.";
        throw std::runtime_error(ss.str());
    }

    ofs << s2 << ", " << s1 << ", " << s0 << std::endl;

    if ( w <= 0 )
    {
        size_t step2 = s1 * s0;
        size_t s2Start, s1Start;

        for ( size_t i = 0; i < s2; ++i )
        {
            ofs << d2 << i << std::endl;

            s2Start = i*step2;

            for ( size_t j = 0; j < s1 ; ++j )
            {
                s1Start = j * s0;

                size_t k;
                for ( k = 0; k < s0 - 1; ++k )
                {
                    ofs << array[s2Start + s1Start + k] << d0;
                }

                ofs << array[s2Start + s1Start + k] << d1;
            }
        }
    }
    else
    {
        size_t step2 = s1 * s0;
        size_t s2Start, s1Start;

        for ( size_t i = 0; i < s2; ++i )
        {
            ofs << d2 << i << std::endl;

            s2Start = i*step2;

            for ( size_t j = 0; j < s1 ; ++j )
            {
                s1Start = j * s0;

                size_t k;
                for ( k = 0; k < s0 - 1; ++k )
                {
                    ofs << std::setw(w) << array[s2Start + s1Start + k] << d0;
                }

                ofs << std::setw(w) << array[s2Start + s1Start + k] << d1;
            }
        }
    }

    ofs.close();
}

#endif //STEREOUTILS_ARRAYOUTPUT_HPP
