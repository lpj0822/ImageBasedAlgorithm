#ifndef GREEDY_NON_MAXIMAL_SUPPRESSION_H
#define GREEDY_NON_MAXIMAL_SUPPRESSION_H

#include "objects_detection_data_structure.h"

/*
I/O:	Name		                                        Type	              Size				Content
[in]	minimalOverlapThreshold			const float		    1	                overlap allowed between two detections be considered different.
                                                                                                                The non maximal suppression is based on greedy variant,
                                                                                                                 see Section 1.2.1 P. Dollar, Integral Channel Features - Addendum, 2009.
                                                                                                                 This overlap is _not_ the PASCAL VOC overlap criterion.
                                                                                                                 0.65 fixed based on the results of P. Dollar 2009 addendum, figure 2.
Realized function:
    + Create a non maximal suppression algorithm!
*/
void greedyNonMaximalSuppression(const DetectorOutputObject* candidateDetections, const float minimalOverlapThreshold, DetectorOutputObject* finalDetections);

#endif // GREEDY_NON_MAXIMAL_SUPPRESSION_H

