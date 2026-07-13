#include <unordered_set>

#include <pcl/common/common.h>

template<typename PointT>
class Ransac {
private:
  int maxIterations;
  float distanceTol;
  int num_points;

public:
  Ransac(int maxIter, float distTol, int nPts) : maxIterations(maxIter), distanceTol(distTol), num_points(nPts) {}
  ~Ransac();
  std::unordered_set<int> Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud);
};

/**********************************************************************/
template<typename PointT>
Ransac<PointT>::~Ransac() {}

/**********************************************************************/
template<typename PointT>
std::unordered_set<int> Ransac<PointT>::Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  std::unordered_set<int> inliersResult;

  auto all_points = cloud->points;

	// walk through all points
	while (maxIterations--) {
		std::unordered_set<int> inliers;
		while (inliers.size()<3) {
			inliers.insert(rand()%num_points);
		}

		// extract 3-points to define plane
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto itr = inliers.begin();
		x1 = all_points[*itr].x; //cloud->points[*itr].x;
		y1 = all_points[*itr].y; //cloud->points[*itr].y;
		z1 = all_points[*itr].z; //cloud->points[*itr].z;
		itr++;
		x2 = all_points[*itr].x;
		y2 = all_points[*itr].y;
		z2 = all_points[*itr].z;
		itr++;
		x3 = all_points[*itr].x;
		y3 = all_points[*itr].y;
		z3 = all_points[*itr].z;

		// define plane equation coefficients
		float a, b, c, d, sqrt_abc;
		a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		d = - (a*x1 + b*y1 + c*z1);
		sqrt_abc = sqrt(a*a + b*b + c*c);

		// Reject non-horizontal (or degenerate) plane hypotheses before they
		// ever compete on inlier count. Without this, "most inliers within
		// distanceTol" is the only criterion, and that's distance/density
		// dependent: at close range, heavy voxel downsampling can leave so
		// few points (~30-40) that a *tilted* plane running through both the
		// true ground and the base of a nearby low object out-scores the
		// real (level) ground plane, sweeping the object into groundCloud.
		// Requiring the normal be close to vertical makes "is this ground"
		// a geometric check, not a point-count one, so it holds the same at
		// 2m or 20m.
		if (sqrt_abc < 1e-6f) {
			continue; // three near-colinear points - no well-defined normal
		}
		const float kMinGroundNormalCos = 0.94f; // ~20 degrees off vertical
		if (fabs(c) / sqrt_abc < kMinGroundNormalCos) {
			continue; // not close to level - not a ground-plane candidate
		}

		// implement RANSAC via point-to-plane distance check
		for (int i=0; i<num_points; i++) {
			if (inliers.count(i)>0) {
				continue;
			}
			// PointT pt = all_points[i];
      PointT pt = all_points[i];
			float dist = fabs(a*pt.x + b*pt.y + c*pt.z + d)/sqrt_abc;

			if (dist<=distanceTol) {
				inliers.insert(i);
			}
		}

		// Compare/copy once per RANSAC iteration (inliers only grows within
		// the for-loop above, so its final size here is already the max for
		// this iteration) - doing this per-point instead was an O(n^2)
		// unordered_set copy once a plane hypothesis picked up many inliers.
		if (inliers.size()>inliersResult.size()) {
			inliersResult = inliers;
		}
	}

  return inliersResult;
}
