#!/usr/bin/env python

'''Averages the distortion/intrisic parameter results stored in the given files,
then prints the result in a format usable by rtslam.
'''

import sys
from xml.etree.ElementTree import ElementTree

def main():
    printAverageResults(*sys.argv[1:])

def printAverageResults(*files):
    avgDistortion, avgIntrinsics = getAverageResults(*files)
    print "Average Distortion: %s" % (', '.join(str(x) for x in avgDistortion))
    print "Average Intrinsics: %s" % (', '.join(str(x) for x in avgIntrinsics))

def getAverageResults(*files):
    distortionResults = []
    intrinsicsResults = []
    for filename in files:
        tree = ElementTree()
        try:
            tree.parse(filename)
        except (OSError, IOError):
            print "unable to read file %s" % filename
            continue

        for distortion in tree.findall('Distortion'):
            # rtslam only takes first 3 distortion coefficients
            distortionResult = tuple(float(x) for x in distortion.find('data').text.split()[:3])
            distortionResults.append(distortionResult)

        for intrinsic in tree.findall('Intrinsics'):
            rawIntrinsics = [float(x) for x in intrinsic.find('data').text.split()]
            # rawIntrinsics will be a flattened matrix of the form:
            # | alphaU   0    u0 |
            # |    0   alphaV v0 |
            # |    0     0     1 |
            # We want u0, v0, alphaU, alphaV - so we want indices 2,5,0,4
            intrinsicsResults.append( (rawIntrinsics[2],
                                       rawIntrinsics[5],
                                       rawIntrinsics[0],
                                       rawIntrinsics[4],
                                      ) )

    from pprint import pprint
    pprint(distortionResults)
    pprint(intrinsicsResults)

    return averageTuples(distortionResults), averageTuples(intrinsicsResults)

def averageTuples(tuples):
    if not tuples:
        return ()
    size = len(tuples[0])
    total = [0] * size
    for element in tuples:
        assert len(element) == size
        for i, val in enumerate(element):
            total[i] += val
    return tuple( x / float(len(tuples)) for x in total )

if __name__ == '__main__':
    main()