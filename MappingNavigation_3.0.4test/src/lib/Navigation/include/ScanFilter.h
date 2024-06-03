#ifndef FILTER_H
#define FILTER_H

#include "scan.h"


void ScanSegmentFilter(CScan *scan, short numSegments);
/*
** filter that reduces number of points
** by replacing points lying in the same segement
** by the scan point which is closest to the range finder.
** Each segment is defined by a cone with a constant angle.
*/

void ScanLineFilter(CScan *scan, int mode);
/* filter that removes scan points which do not ly on a line */

void ScanLineFilterFast(CScan *scan);
/* like ScanLineFilter but assumes m_nLineID is already set correctly */

void ScanNotLineFilterFast(CScan *scan);
/* removes all points on a line, assumes m_nLineID is set correctly */

void ScanNotLineButCloseFilterFast(CScan *scan, float maxDist);
/* removes all points on a line and those that are far from a line,
   assumes m_nLineID is set correctly */

void ScanClusterFilter(CScan *scan, float radius, int minNum);
/* removes all points whose neighbors within radius make a cluster
   of less than minNum points.  This removes single points */

float ScanCommonPointsFilter(CScan *scan1, CScan *scan2,
    float max_dist);
/*
** filter that removes scan points of scan1 which have no correspondence
** partner in the other scan.  The correspondence partner is chosen
** by searching for a point within max_dist distance.
*/

void ScanCommonPointsFilter2(CScan *scan1, CScan *scan2, 
    float max_dist, float *reduced1, float *reduced2);
/*
** Applies ScanCommonPointsFilter to both scans.
*/

float ScanProjectionFilter(CScan *scan, float scanX, float scanY,
    CScan *frame, float maxred);
/*
** filter that removes scan points which are not visible
** from another location.  If frame != NULL then points which lie
** behind the frame scan are removed, too.
** In maxred you specify a maximum reduction of scan points.
** If more points then maxred * num_points have to be removed
** then a reduction value larger than maxred is returned.
** The scan might not be changed in this case.
** Returns percentage [0 .. 1] of points removed.
*/


#endif

