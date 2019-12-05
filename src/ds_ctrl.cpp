#include "ds_ctrl.h"

using namespace std;

DataSourceCtrl::DataSourceCtrl() {}
DataSourceCtrl::DataSourceCtrl(vector<DataSource*> ds_){
    this->ds = ds_;
}

void DataSourceCtrl::read(){
    for(DataSource* d : ds){
        d->readSensor();
    }
}
void DataSourceCtrl::update(){
    read();
    postProcess();
}

void DataSourceCtrl::postProcess(){
    for(DataSource* d : ds){
        d->postProcess();
    }
}

void DataSourceCtrl::test(){
    DEBUG.println("========================================");
    for(DataSource* d : ds){
        d->test();
    }
    DEBUG.println("========================================");

}