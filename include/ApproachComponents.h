#ifndef MULTIVIEWREFINEMENT_H
#define MULTIVIEWREFINEMENT_H

#include <vector>
#include <PointCloud.h>

class ApproachComponents
{
public:
    ApproachComponents();

    static void preprocessing(std::vector< std::shared_ptr<PointCloud> >& frames, int step, bool showDepthMap, int limit);

    static void pairwiseAlignment(std::vector<std::shared_ptr<PointCloud> > &frames, int nFrames);

    static void pairwiseAlignmentAndRefinement(std::vector<std::shared_ptr<PointCloud> > &frames, int nFrames, float cutoff);

    static void pairwiseRefinement(std::vector<std::shared_ptr<PointCloud> > &frames, float cutoff);

    static void g2oOptimizer(std::vector< std::shared_ptr<PointCloud> >& frames, float cutoff, int iter=5, int knn=5, double huberWidth=-1, bool useLevenberg=false);

    static bool stopFrames;


private:
    static void alignFrame(std::vector<std::shared_ptr<PointCloud> > &frames, int nFrames, int i_frame);

};

#endif // MULTIVIEWREFINEMENT_H
