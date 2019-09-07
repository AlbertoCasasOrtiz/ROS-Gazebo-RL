//
// Created by alberto on 6/09/19.
//

#include "Table.h"
#include "Actions.h"
#include <ros/ros.h>
#include <fstream>

void Table::initializeTable(State state) {
    std::vector<float> actions;
    actions.reserve(Actions::size);
    for(int i = 0; i < Actions::size+1; i++)
        actions.push_back(0.0);

    Table::table.emplace(state, actions);
}
void Table::setValue(State state, int a, float value) {
    // Check if the point is in the table..
    auto it = Table::table.find(state);

    if(it == Table::table.end()) {
        //If not in table, insert.
        Table::addState(state);
    }
    Table::table.at(state).at(a) =  value;
}

float Table::getValue(State state, int a) {
    return Table::table.at(state).at(a);
}

std::vector<int> Table::getSizesTable() {
    std::vector<int> res;
    res.push_back(Table::table.size());
    res.push_back(Actions::size);
    return res;
}

void Table::addState(State state){
    //If not in table, insert.
    std::vector<float> actions;
    actions.reserve(Actions::size);
    for(int i = 0; i < Actions::size; i++)
        actions.push_back(0.0);

    Table::table.emplace(state, actions);
}

void Table::printTable(std::string name) {
    std::ofstream file(name);

    for(std::map<State, std::vector<float>>::const_iterator it = Table::table.begin(); it != Table::table.end(); ++it)
    {
        file << it->first.p.x << " " << it->first.p.y << "-" << it->second.at(0) << " " << it->second.at(1) << " " << it->second.at(2) << " " << it->second.at(3) << "\n";
    }
    file.close();
}
