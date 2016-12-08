#include <gtest/gtest.h>
#include <amcl/sensors/amcl_odom.h>
#include <amcl/map/map.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/assert.h> // ROS_ASSERT

using namespace amcl;

pf_vector_t constantPoseGenerator(void* arg)
{
  pf_vector_t p;
  p.v[0] = 0.0;
  p.v[1] = 0.0;
  p.v[2] = 0.0;
  return p;
}


map_t* createEmptyMap(){
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = 10.0;
  map->size_y = 20.0;
  map->scale = 0.1;
  map->origin_x = 5.0;
  map->origin_y = 10.0;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
/*  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(map_msg.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if(map_msg.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }*/

  return map;
}

TEST(OdometryModelTest, withZeroAlphasParticlesMoveExactlyFollowingOdom){
	int min_samples = 3;
	int max_samples = 10;
	double alpha_slow = 0.1;
	double alpha_fast = 0.01;
	pf_init_model_fn_t random_pose_fn = (pf_init_model_fn_t)constantPoseGenerator;
	map_t* map = createEmptyMap();
	void* random_pose_data =(void *)map;
	pf_t* pf = pf_alloc(min_samples, max_samples, alpha_slow, alpha_fast, random_pose_fn, random_pose_data);
	// Check that all samples are initialized to (0,0,0)
	pf_sample_set_t* set = pf->sets + pf->current_set;
        for (int i = 0; i < set->sample_count; i++){
		pf_sample_t* sample = set->samples + i;
		EXPECT_EQ(sample->pose.v[0],0.0);
		EXPECT_EQ(sample->pose.v[1],0.0);
		EXPECT_EQ(sample->pose.v[2],0.0);
	}

	// We move by x+=1
	AMCLOdomData data;
	data.pose.v[0]=6.0;
	data.pose.v[1]=10.0;
	data.pose.v[2]=0.0;
	data.delta.v[0]=1.0;
	data.delta.v[1]=0.0;
	data.delta.v[2]=0.0;
	AMCLOdom amcl_odom;
	double alpha1 = 0.0;
	double alpha2 = 0.0;
	double alpha3 = 0.0;
	double alpha4 = 0.0;
	double alpha5 = 0.0;
	amcl_odom.SetModel(ODOM_MODEL_DIFF, alpha1, alpha2, alpha3, alpha4, alpha5);
        ASSERT_TRUE(amcl_odom.UpdateAction(pf, &data));
	set = pf->sets + pf->current_set;
        for (int i = 0; i < set->sample_count; i++){
		pf_sample_t* sample = set->samples + i;
		EXPECT_EQ(sample->pose.v[0],1.0);
		EXPECT_EQ(sample->pose.v[1],0.0);
		EXPECT_EQ(sample->pose.v[2],0.0);
	}
}
