
#include <stdio.h>
#include <octomap/octomap_timing.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include "testing.h"

using namespace std;
using namespace octomap;
using namespace octomath;


void printTreeStats(const OcTree<true>& tree, const OcTree<true>& tree_copy) {
  cout << "Main OcTree:   " << tree.calcNumUniqueNodes() << " of " << tree.calcNumNodes() << " are unique; "
       << tree.getNumUniqueLeafNodes() << " of " << tree.getNumLeafNodes() << " leaf nodes are unique." << endl
       << "Copied OcTree: " << tree_copy.calcNumUniqueNodes() << " of " << tree_copy.calcNumNodes() << " are unique; "
       << tree_copy.getNumUniqueLeafNodes() << " of " << tree_copy.getNumLeafNodes() << " leaf nodes are unique." << endl;
}

double timediff(const timeval& start, const timeval& stop){
  return (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);
}

int main(int argc, char** argv) {


  {
    OcTree<true> basic_tree (0.05);
    point3d origin (0.0f, 0.0f, 0.0f);
    point3d pt (2.01f, 0.01f, 0.01f);
    cout << "Adding point at " << pt << " ..." << endl;
    Pointcloud p;
    p.push_back(pt);
    basic_tree.insertPointCloud(p, origin);

    OcTree<true> basic_tree_copy (basic_tree);
    printTreeStats(basic_tree, basic_tree_copy);
    EXPECT_TRUE(basic_tree == basic_tree_copy);

    cout << "Using some const iterators..." << endl;
    unsigned leaf_counter = 0;
    unsigned node_counter = 0;
    for (OcTree<true>::const_tree_iterator it = basic_tree.cbegin_tree(), end_it = basic_tree.cend_tree();
         it != end_it; ++it) {
      ++node_counter;
    }
    for (OcTree<true>::const_iterator it = basic_tree.cbegin(), end_it = basic_tree.cend();
         it != end_it; ++it) {
      ++leaf_counter;
    }
    cout << "Counted " << node_counter << " nodes and " << leaf_counter << " leaves." << endl;
    printTreeStats(basic_tree, basic_tree_copy);
    EXPECT_TRUE(basic_tree.calcNumUniqueNodes() == 0);
    EXPECT_TRUE(basic_tree_copy.calcNumUniqueNodes() == 0);

    point3d pt2 (-2.01f, 0.01f, 0.01f);
    cout << "Adding point at " << pt2 << " ..." << endl;
    Pointcloud p2;
    p2.push_back(pt2);
    basic_tree.insertPointCloud(p2, origin);

    printTreeStats(basic_tree, basic_tree_copy);

    cout << "Copying again..." << endl;
    // Yes, this is ugly. So sue me.
    basic_tree_copy.~OcTree<true>();
    new (&basic_tree_copy) OcTree<true>(basic_tree);

    EXPECT_TRUE(basic_tree == basic_tree_copy);
    printTreeStats(basic_tree, basic_tree_copy);

    cout << "Using non-const iterator..." << endl;
    leaf_counter = 0;
    OcTree<true>::const_iterator cit = basic_tree_copy.cbegin(), end_cit = basic_tree_copy.cend();
    for (OcTree<true>::const_iterator it = basic_tree.cbegin(), end_it = basic_tree.cend();
         it != end_it; ++it, ++cit) {
      EXPECT_TRUE(cit->getLogOdds() == it->getLogOdds());
    }
    cit = basic_tree_copy.cbegin(); end_cit = basic_tree_copy.cend();
    for (OcTree<true>::iterator it = basic_tree.begin(), end_it = basic_tree.end();
         it != end_it; ++it, ++cit) {
      ++leaf_counter;
      float clo = cit->getLogOdds();
      it->setLogOdds(0.357); // This should leave a trace!
      EXPECT_TRUE(clo == cit->getLogOdds());
      EXPECT_TRUE(clo != it->getLogOdds());
    }
    cit = basic_tree_copy.cbegin(); end_cit = basic_tree_copy.cend();
    for (OcTree<true>::const_iterator it = basic_tree.cbegin(), end_it = basic_tree.cend();
         it != end_it; ++it, ++cit) {
      EXPECT_TRUE(cit->getLogOdds() != it->getLogOdds());
    }
    EXPECT_TRUE(!(basic_tree == basic_tree_copy));
    cout << "Counted " << leaf_counter << " leaves." << endl;
    printTreeStats(basic_tree, basic_tree_copy);

    EXPECT_TRUE(basic_tree.calcNumUniqueNodes() == basic_tree.calcNumNodes());
    EXPECT_TRUE(basic_tree_copy.calcNumUniqueNodes() == basic_tree_copy.calcNumNodes());
  }


  OcTree<true> tree (0.05);



  //  point3d origin (10.01, 10.01, 10.02);
  point3d origin (0.01f, 0.01f, 0.02f);
  point3d point_on_surface (2.01f, 0.01f, 0.01f);

  cout << "Generating sphere at " << origin << " ..." << endl;

  unsigned sphere_beams = 500;
  double angle = 2.0*M_PI/double(sphere_beams);
  Pointcloud p;
  for (unsigned i=0; i<sphere_beams; i++) {
    for (unsigned j=0; j<sphere_beams; j++) {
      p.push_back(origin+point_on_surface);
      point_on_surface.rotate_IP (0,0,angle);
    }
    point_on_surface.rotate_IP (0,angle,0);
  }
  tree.insertPointCloud(p, origin);


  cout << "Writing to sphere_cow.bt..." << endl;
  EXPECT_TRUE(tree.writeBinary("sphere_cow.bt"));

  // -----------------------------------------------

  cout << "Casting rays in sphere ..." << endl;

  OcTree<true> sampled_surface (0.05);  

  point3d direction = point3d (1.0,0.0,0.0);
  point3d obstacle(0,0,0);

  unsigned int hit (0);
  unsigned int miss (0);
  unsigned int unknown (0);
  double mean_dist(0);

  for (unsigned i=0; i<sphere_beams; i++) {
    for (unsigned j=0; j<sphere_beams; j++) {
      if (!tree.castRay(origin, direction, obstacle, false, 3.)) {
        // hit unknown
        if (!tree.search(obstacle))
          unknown++;
        else
          miss++;
      }
      else {
        hit++;
        mean_dist += (obstacle - origin).norm();
        sampled_surface.updateNode(obstacle, true);
      }
      direction.rotate_IP (0,0,angle);
    }
    direction.rotate_IP (0,angle,0);
  }

  cout << "Writing sampled_surface_cow.bt" << endl;
  EXPECT_TRUE(sampled_surface.writeBinary("sampled_surface_cow.bt"));

  mean_dist /= (double) hit;
  std::cout << " hits / misses / unknown: " << hit  << " / " << miss << " / " << unknown << std::endl;
  std::cout << " mean obstacle dist: " << mean_dist << std::endl;
  EXPECT_NEAR(mean_dist, 2., 0.05);
  EXPECT_EQ(hit, (sphere_beams*sphere_beams));
  EXPECT_EQ(miss, 0);
  EXPECT_EQ(unknown, 0);


  // -----------------------------------------------

  cout << "generating single rays..." << endl;
  OcTree<true> single_beams(0.03333);
  int num_beams = 17;
  float beamLength = 10.0f;
  point3d single_origin (1.0f, 0.45f, 0.45f);
  point3d single_origin_top (1.0f, 0.45f, 1.0);
  point3d single_endpoint(beamLength, 0.0f, 0.0f);


  for (int i=0; i<num_beams/2; i++) {
    for (int j=0; j<num_beams; j++) {
      if (!single_beams.insertRay(single_origin, single_origin+single_endpoint)) {
	cout << "ERROR while inserting ray from " << single_origin << " to " << single_endpoint << endl;
      }
      single_endpoint.rotate_IP (0,0,DEG2RAD(360.0/num_beams));
    }
    single_endpoint.rotate_IP (0,DEG2RAD(360.0/num_beams),0);
  }


  cout << "done." << endl;

  cout << "creating CoW copy..." << endl;
  OcTree<true> single_beams_copy(single_beams);
  EXPECT_TRUE(single_beams.calcNumUniqueNodes() == 0);
  EXPECT_TRUE(single_beams_copy.calcNumUniqueNodes() == 0);
  unsigned int nodes_in_copy = single_beams_copy.calcNumNodes();

  printTreeStats(single_beams, single_beams_copy);

  cout << "generating second half of single rays..." << endl;
  for (int i=num_beams/2; i<num_beams; i++) {
    for (int j=0; j<num_beams; j++) {
      if (!single_beams.insertRay(single_origin, single_origin+single_endpoint)) {
	cout << "ERROR while inserting ray from " << single_origin << " to " << single_endpoint << endl;
      }
      single_endpoint.rotate_IP (0,0,DEG2RAD(360.0/num_beams));
    }
    single_endpoint.rotate_IP (0,DEG2RAD(360.0/num_beams),0);
  }
  printTreeStats(single_beams, single_beams_copy);
  cout << "Copied OcTree: " << single_beams_copy.uniqueMemoryUsage() << " bytes used uniquely (vs. " << single_beams_copy.memoryUsage() << " for the whole tree)." << endl;

  EXPECT_TRUE(single_beams_copy.calcNumNodes() == nodes_in_copy);
  cout << "writing to beams_cow.bt..." << endl;
  EXPECT_TRUE(single_beams_copy.writeBinary("beams_cow.bt"));
  cout << "writing to beams_cow_2.bt..." << endl;
  EXPECT_TRUE(single_beams.writeBinary("beams_cow_2.bt"));
  EXPECT_TRUE(single_beams.calcNumNodes() == single_beams.calcNumUniqueNodes());

  ////// more tests from unit_tests.cpp:
  double res = 0.1;
  double res_2 = res/2.0;
  OcTree<true> cubeTree(res);
  // fill a cube with "free", end is "occupied":
  for (float x=-0.95; x <= 1.0; x+=res){
    for (float y=-0.95; y <= 1.0; y+=res){
      for (float z=-0.95; z <= 1.0; z+=res){
        if (x < 0.9){
          EXPECT_TRUE(cubeTree.updateNode(point3d(x,y,z), false));
        } else{
          EXPECT_TRUE(cubeTree.updateNode(point3d(x,y,z), true));
        }
      }
    }
  }

  // fill some "floor":
  EXPECT_TRUE(cubeTree.updateNode(res_2,res_2,-res_2, true));
  EXPECT_TRUE(cubeTree.updateNode(3*res_2,res_2,-res_2, true));
  EXPECT_TRUE(cubeTree.updateNode(-res_2,res_2,-res_2, true));
  EXPECT_TRUE(cubeTree.updateNode(-3*res_2,res_2,-res_2, true));

  cubeTree.writeBinary("raycasting_cube_cow.bt");
  origin = point3d(0.0f, 0.0f, 0.0f);
  point3d end;
  // hit the corner:
  direction = point3d(0.95f, 0.95f, 0.95f);
  EXPECT_TRUE(cubeTree.castRay(origin, direction, end, false));
  EXPECT_TRUE(cubeTree.isNodeOccupied(cubeTree.search(end)));
  std::cout << "Hit occupied voxel: " << end << std::endl;
  direction = point3d(1.0, 0.0, 0.0);
  EXPECT_TRUE(cubeTree.castRay(origin, direction, end, false));
  EXPECT_TRUE(cubeTree.isNodeOccupied(cubeTree.search(end)));
  std::cout << "Hit occupied voxel: " << end << std::endl;
  EXPECT_NEAR(1.0, (origin - end).norm(), res_2);

  // hit bottom:
  origin = point3d(res_2, res_2, 0.5f);
  direction = point3d(0.0, 0.0, -1.0f);
  EXPECT_TRUE(cubeTree.castRay(origin, direction, end, false));
  EXPECT_TRUE(cubeTree.isNodeOccupied(cubeTree.search(end)));
  std::cout << "Hit voxel: " << end << std::endl;
  EXPECT_FLOAT_EQ(origin(0), end(0));
  EXPECT_FLOAT_EQ(origin(1), end(1));
  EXPECT_FLOAT_EQ(-res_2, end(2));


  // hit boundary of unknown:
  origin = point3d(0.0f, 0.0f, 0.0f);
  direction = point3d(0.0f, 1.0f, 0.0f);
  EXPECT_FALSE(cubeTree.castRay(origin, direction, end, false));
  EXPECT_FALSE(cubeTree.search(end));
  std::cout << "Boundary unknown hit: " << end << std::endl;

  // hit boundary of octree:
  EXPECT_FALSE(cubeTree.castRay(origin, direction, end, true));
  EXPECT_FALSE(cubeTree.search(end));
  EXPECT_FLOAT_EQ(end.x(), res_2);
  EXPECT_FLOAT_EQ(end.y(), float(32768*res-res_2));
  EXPECT_FLOAT_EQ(end.z(), res_2);
  direction = point3d(-1.0f, 0.0f, 0.0f);
  EXPECT_FALSE(cubeTree.castRay(origin, direction, end, true));
  EXPECT_FALSE(cubeTree.search(end));
  EXPECT_FLOAT_EQ(end.x(), float(-32767*res-res_2));
  EXPECT_FLOAT_EQ(end.y(), res_2);
  EXPECT_FLOAT_EQ(end.z(), res_2);

  // test maxrange:
  EXPECT_FALSE(cubeTree.castRay(origin, direction, end, true, 0.9));
  std::cout << "Max range endpoint: " << end << std::endl;
  OcTreeNode<true>* endPt = cubeTree.search(end);
  EXPECT_TRUE(endPt);
  EXPECT_FALSE(cubeTree.isNodeOccupied(endPt));
  double dist = (origin - end).norm();
  EXPECT_NEAR(0.9, dist, res);


  // -----------------------------------------------

  timeval start;
  timeval stop;
  unsigned ray_beams = 180;
  {
    cout << "OcTree<false>: generating rays..." << endl;
    OcTree<false> tree(0.05);
    float beamLength = 10.0f;
    point3d origin (1.0f, 0.45f, 0.45f);
    point3d origin_top (1.0f, 0.45f, 1.0);
    point3d endpoint(beamLength, 0.0f, 0.0f);

    gettimeofday(&start, NULL);  // start timer
    for (unsigned int i=0; i<ray_beams; i++) {
      for (unsigned int j=0; j<ray_beams; j++) {
        if (!tree.insertRay(origin, origin+endpoint)) {
          cout << "ERROR while inserting ray from " << origin << " to " << endpoint << endl;
        }
        endpoint.rotate_IP (0,0,DEG2RAD(360.0/ray_beams));
      }
      endpoint.rotate_IP (0,DEG2RAD(360.0/ray_beams),0);
    }
    gettimeofday(&stop, NULL);  // stop timer
    cout << "done (" << timediff(start, stop) << " seconds)" << endl;

    cout << "OcTree<false>: creating copy..." << endl;
    gettimeofday(&start, NULL);  // start timer
    OcTree<false> tree_copy(tree);
    gettimeofday(&stop, NULL);  // stop timer
    cout << "done (" << timediff(start, stop) << " seconds)" << endl;

    cout << "OcTree<false>: re-generating rays after copy..." << endl;
    endpoint = point3d(beamLength, 0.0f, 0.0f);

    gettimeofday(&start, NULL);  // start timer
    for (unsigned int i=0; i<ray_beams; i++) {
      for (unsigned int j=0; j<ray_beams; j++) {
        if (!tree_copy.insertRay(origin, origin+endpoint)) {
          cout << "ERROR while inserting ray from " << origin << " to " << endpoint << endl;
        }
        endpoint.rotate_IP (0,0,DEG2RAD(360.0/ray_beams));
      }
      endpoint.rotate_IP (0,DEG2RAD(360.0/ray_beams),0);
    }
    gettimeofday(&stop, NULL);  // stop timer
    cout << "done (" << timediff(start, stop) << " seconds)" << endl;
  }

  {
    cout << "OcTree<true>: generating rays..." << endl;
    OcTree<true> tree(0.05);
    float beamLength = 10.0f;
    point3d origin (1.0f, 0.45f, 0.45f);
    point3d origin_top (1.0f, 0.45f, 1.0);
    point3d endpoint(beamLength, 0.0f, 0.0f);

    gettimeofday(&start, NULL);  // start timer
    for (unsigned int i=0; i<ray_beams; i++) {
      for (unsigned int j=0; j<ray_beams; j++) {
        if (!tree.insertRay(origin, origin+endpoint)) {
          cout << "ERROR while inserting ray from " << origin << " to " << endpoint << endl;
        }
        endpoint.rotate_IP (0,0,DEG2RAD(360.0/ray_beams));
      }
      endpoint.rotate_IP (0,DEG2RAD(360.0/ray_beams),0);
    }
    gettimeofday(&stop, NULL);  // stop timer
    cout << "done (" << timediff(start, stop) << " seconds)" << endl;

    cout << "OcTree<true>: creating copy..." << endl;
    gettimeofday(&start, NULL);  // start timer
    OcTree<true> tree_copy(tree);
    gettimeofday(&stop, NULL);  // stop timer
    cout << "done (" << timediff(start, stop) << " seconds)" << endl;

    cout << "OcTree<true>: re-generating rays after copy..." << endl;
    endpoint = point3d(beamLength, 0.0f, 0.0f);

    gettimeofday(&start, NULL);  // start timer
    for (unsigned int i=0; i<ray_beams; i++) {
      for (unsigned int j=0; j<ray_beams; j++) {
        if (!tree_copy.insertRay(origin, origin+endpoint)) {
          cout << "ERROR while inserting ray from " << origin << " to " << endpoint << endl;
        }
        endpoint.rotate_IP (0,0,DEG2RAD(360.0/ray_beams));
      }
      endpoint.rotate_IP (0,DEG2RAD(360.0/ray_beams),0);
    }
    gettimeofday(&stop, NULL);  // stop timer
    cout << "done (" << timediff(start, stop) << " seconds)" << endl;
  }

  std::cout << "Test successful\n";
  return 0;
}
