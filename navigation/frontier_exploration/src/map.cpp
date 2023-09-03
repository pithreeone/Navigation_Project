#include <string.h>
#include <frontier_exploration/map.h>

Map::Map(nav_msgs::OccupancyGrid map, int thresh)
{
    grid_map_ = map;
    thresh_ = thresh;
    res_ = map.info.resolution;
    indicator_map_ = new Point_Indicator*[map.info.width];
    for(int i=0; i<map.info.width; i++){
        indicator_map_[i] = new Point_Indicator[map.info.height];
    }
    for(int i=0; i<getWidth(); i++){
        for(int j=0;j<getHeight(); j++){
            indicator_map_[i][j] = Point_Indicator::UDF;
        }
    }
};


int Map::getNeighborData(int idx, int idy, Direction dir)
{
    if(dir == Direction::Front){
        if(idy <= grid_map_.info.height - 2){
            return grid_map_.data[(idy+1) * grid_map_.info.width + idx];
        }
    }else if(dir == Direction::Back){
        if(idy >= 1){
            return grid_map_.data[(idy-1) * grid_map_.info.width + idx];
        }
    }else if(dir == Direction::Right){
        if(idx <= grid_map_.info.width - 2){
            return grid_map_.data[idy * grid_map_.info.width + idx + 1];
        }
    }else if(dir == Direction::Left){
        if(idx >= 1){
            return grid_map_.data[idy * grid_map_.info.width + idx - 1];
        }
    }else{
        return -2;
    }
    return -2;
}

bool Map::ifFrontierPoint(int idx, int idy)
{   
    int value = getCellData(idx, idy);
    if(value != -1)
        return false;
    if((getNeighborData(idx, idy, Direction::Front) <= thresh_ && getNeighborData(idx, idy, Direction::Front) >= 0) ||
       (getNeighborData(idx, idy, Direction::Back) <= thresh_ && getNeighborData(idx, idy, Direction::Back) >= 0) ||
       (getNeighborData(idx, idy, Direction::Left) <= thresh_ && getNeighborData(idx, idy, Direction::Left) >= 0) ||
       (getNeighborData(idx, idy, Direction::Right) <= thresh_ && getNeighborData(idx, idy, Direction::Right) >= 0)){
        return true;
    }

    return false;
}

bool Map::ifOpenSpacePoint(int idx, int idy)
{
    int value = getCellData(idx, idy);
    if(value >=0 && value <= thresh_)
        return true;
    return false;
}

Point_Indicator Map::getNeighborIndicator(int idx, int idy, Direction dir)
{
    if(dir == Direction::Front){
        return indicator_map_[idx][idy+1];
    }else if(dir == Direction::Left){
        return indicator_map_[idx-1][idy];
    }else if(dir == Direction::Back){
        return indicator_map_[idx][idy-1];
    }else if(dir == Direction::Right){
        return indicator_map_[idx+1][idy];
    }
}

bool Map::ifHasMapOpenSpaceNeighbor(int idx, int idy)
{
    bool ans = false;
    for(int dir=int(Direction::Front); dir<=int(Direction::Right); dir++){
        if(dir == int(Direction::Front)){
            if(idy <= grid_map_.info.height - 2){
                ans |= ifOpenSpacePoint(idx, idy+1);
            }
        }else if(dir == int(Direction::Back)){
            if(idy >= 1){
                ans |= ifOpenSpacePoint(idx, idy-1);
            }
        }else if(dir == int(Direction::Right)){
            if(idx <= grid_map_.info.width - 2){
                ans |= ifOpenSpacePoint(idx+1, idy);
            }
        }else if(dir == int(Direction::Left)){
            if(idx >= 1){
                ans |= ifOpenSpacePoint(idx-1, idy);
            }
        }
    }
    return ans;
}

