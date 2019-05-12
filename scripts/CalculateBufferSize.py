
from __future__ import print_function

import argparse

SIZE_GB = 1024.0 * 1024 * 1024

if __name__ == "__main__":
    print("Calculate the buffer size.")

    parser = argparse.ArgumentParser(description="Estimate the buffer size needed for SGM method.")

    parser.add_argument("height", type=int, \
        help="The row numbers")
    parser.add_argument("width", type=int, \
        help="The column numbers.")
    parser.add_argument("d", type=int, \
        help="The number of disparities.")
    parser.add_argument("--type-size", type=int, default=2, \
        help="The sizeof() value of the type used.")

    args = parser.parse_args()

    sizeLeftCostImage  = args.height * args.width * args.d * args.type_size
    sizeRightCostImage = args.height * args.width * args.d * args.type_size

    sizePixelwiseCostRow  = args.width * args.d
    sizeRowAggrecatedCost = args.width * args.d * ( 2 * 2 + 2 ) * args.type_size

    widthStep = args.width + 15 - ( args.width - 1 ) % 16
    sizeHalfPixelRightMin = widthStep
    sizeHalfPixelRightMax = widthStep

    disparitySize = args.d + 16
    pathTotal     = 8
    pathMinCostBufferSize = ( args.width + 2 ) * pathTotal
    pathDisparitySize     = pathTotal * disparitySize
    pathCostBufferSize    = pathMinCostBufferSize * disparitySize
    costSumBufferRowSize  = args.width * args.d
    costSumBufferSize     = costSumBufferRowSize * args.height

    totalBufferSize = ( pathMinCostBufferSize + pathCostBufferSize ) * 2 + costSumBufferSize + 16
    sizeTotalBufferByte = totalBufferSize * args.type_size

    total = sizeLeftCostImage + sizeRightCostImage + \
        sizePixelwiseCostRow + sizeRowAggrecatedCost + \
        sizeHalfPixelRightMin + sizeHalfPixelRightMax + \
        sizeTotalBufferByte

    # Show the info.
    print( "sizeLeftCostImage     = %d( %f GB )" % ( sizeLeftCostImage, sizeLeftCostImage / SIZE_GB ) )
    print( "sizeRightCostImage    = %d( %f GB )" % ( sizeRightCostImage, sizeRightCostImage / SIZE_GB ) )
    print( "sizePixelwiseCostRow  = %d( %f GB )" % ( sizePixelwiseCostRow, sizePixelwiseCostRow / SIZE_GB ) )
    print( "sizeRowAggrecatedCost = %d( %f GB )" % ( sizeRowAggrecatedCost, sizeRowAggrecatedCost / SIZE_GB ) )
    print( "sizeHalfPixelRightMin = %d( %f GB )" % ( sizeHalfPixelRightMin, sizeHalfPixelRightMin / SIZE_GB ) )
    print( "sizeHalfPixelRightMax = %d( %f GB )" % ( sizeHalfPixelRightMax, sizeHalfPixelRightMax / SIZE_GB ) )
    print( "sizeTotalBufferByte   = %d( %f GB )" % ( sizeTotalBufferByte, sizeTotalBufferByte / SIZE_GB ) )
    print( "total                 = %d( %f GB )" % ( total, total/SIZE_GB ))
