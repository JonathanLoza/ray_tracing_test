#include <cstdio>
#include <iostream>
#include <string.h>
#include <vector>
#include <chrono>
#include <omp.h>

#include <bvh/bvh.hpp>
#include <bvh/sweep_sah_builder.hpp>
#include "bvh/linear_bvh_builder.hpp"
#include "bvh/binned_sah_builder.hpp"
#include "bvh/locally_ordered_clustering_builder.hpp"
#include <bvh/vector.hpp>
#include <bvh/triangle.hpp>

using Scalar = float;
using Vector3  = bvh::Vector3<Scalar>;
using Triangle = bvh::Triangle<Scalar>;
using Bvh      = bvh::Bvh<Scalar>;

std::vector<Triangle> triangles;

bool load( const char* path ){
  FILE * file = fopen(path, "r");
  if( file == NULL ){
      printf("Impossible to open the file !\n");
      return false;
  }
  printf("loaded\n");
  std::vector<Vector3> temp_vertices;
  std::vector<std::vector<unsigned int>> faces;
  while( 1 ){
    char lineHeader[128];
    // read the first word of the line
    int res = fscanf(file, "%s", lineHeader);
    if (res == EOF)
        break; // EOF = End Of File. Quit the loop. 
    if ( strcmp( lineHeader, "v" ) == 0 ){
      Scalar x,y,z;
      fscanf(file, "%f %f %f\n", &x, &y, &z );
      //std::cout<<x<<" "<<y<<" "<<z<<std::endl;
      temp_vertices.push_back(Vector3(x,y,z));
    }else if ( strcmp( lineHeader, "f" ) == 0 ){
      unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
      //int matches = fscanf(file, "%d %d %d\n", &vertexIndex[0],&vertexIndex[1],&vertexIndex[2]);
      //int matches = fscanf(file, "%d//%d %d//%d %d//%d\n", &vertexIndex[0], &normalIndex[0], &vertexIndex[1], &normalIndex[1], &vertexIndex[2], &normalIndex[2] );
      int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2] );
      if (matches != 9){
          printf("File can't be read by our simple parser : ( Try exporting with other options\n");
          return false;
      }
      faces.push_back(std::vector<unsigned int>{vertexIndex[0],vertexIndex[1],vertexIndex[2]});
    }
  }

  std::cout<<"#Vertices: "<<temp_vertices.size()<<std::endl;
  unsigned int v1,v2,v3;
  for(auto t:faces){
    v1=t[0]-1;
    v2=t[1]-1;
    v3=t[2]-1;
    triangles.emplace_back(temp_vertices[v1],temp_vertices[v2],temp_vertices[v3]);
  }
  fclose(file);
  return true;
}

int main(){
  load("../scenes/rt/rungholt/rungholt2.obj");
  std::cout<<"#Triangles: "<<triangles.size()<<std::endl;
  //omp_set_num_threads(120);

  auto start = std::chrono::steady_clock::now();
  Bvh bvh;
  bvh::BinnedSahBuilder<Bvh,4> builder(bvh);
  //bvh::SweepSahBuilder<Bvh> builder(bvh);
  //bvh::LinearBvhBuilder<Bvh,size_t> builder(bvh);
  //bvh::LocallyOrderedClusteringBuilder<Bvh,size_t> builder(bvh);builder.search_radius=5;
  auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(triangles.data(), triangles.size());
  auto global_bbox = bvh::compute_bounding_boxes_union(bboxes.get(), triangles.size());
  builder.build(global_bbox, bboxes.get(), centers.get(), triangles.size());
  auto end = std::chrono::steady_clock::now();
  std::cout << "Elapsed time in milliseconds: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
        << " ms" <<std::endl;
  return 0;
}
