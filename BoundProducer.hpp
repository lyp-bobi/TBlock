//
// Created by Chuang on 2022/9/16.
//

#ifndef TBLOCK_BOUNDPRODUCER_HPP
#define TBLOCK_BOUNDPRODUCER_HPP

#include "postgis.h"

#include "Bound.h"

class BOUNDPRODUCER
{
private:
    POSTGIS_POINTARRAY *m_ptarray;
public:
    BOUNDPRODUCER(POSTGIS_POINTARRAY* p): m_ptarray(p){};
    BOUNDLIST* produce_tbox_list(int numseg);
    BOUNDLIST* produce_tblock_list(int numseg);
};


#endif //TBLOCK_BOUNDPRODUCER_HPP
