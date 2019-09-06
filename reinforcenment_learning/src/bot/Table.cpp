//
// Created by alberto on 6/09/19.
//

#include "Table.h"
#include "Actions.h"

void Table::initializeTableQ(State state) {
    std::vector<float> actions;
    actions.reserve(Actions::size);
    for(int i = 0; i < Actions::size; i++)
        actions.push_back(0.0);

    Table::tableQ.emplace(state, actions);
}

void Table::initializeTableE(State state) {
    std::vector<float> actions;
    actions.reserve(Actions::size);
    for(int i = 0; i < Actions::size; i++)
        actions.push_back(0.0);

    Table::tableQ.emplace(state, actions);
}

void Table::updateValueQ(State state, int a, float value) {
    // Check if the point is in the table..
    auto it = Table::tableQ.find(state);

    if(it == Table::tableQ.end()) {
        //If not in table, insert.
        std::vector<float> actions;
        actions.reserve(Actions::size);
        for(int i = 0; i < Actions::size; i++)
            actions.push_back(0.0);

        Table::tableQ.emplace(state, actions);
    } else
        //If in table, update.
        Table::tableQ.at(state).at(a) = Table::tableQ.at(state).at(a) + value;
}

void Table::updateValueE(State state, int a, float value) {
    // Check if the point is in the table..
    auto it = Table::tableE.find(state);

    if(it == Table::tableE.end()) {
        //If not in table, insert.
        std::vector<float> actions;
        actions.reserve(Actions::size);
        for(int i = 0; i < Actions::size; i++)
            actions.push_back(0.0);

        Table::tableE.emplace(state, actions);
    } else
        //If in table, update.
        Table::tableE.at(state).at(a) = Table::tableE.at(state).at(a) + value;
}

float Table::getValueQ(State state, int a) {
    return Table::tableQ.at(state).at(a);
}

float Table::getValueE(State state, int a) {
    return Table::tableE.at(state).at(a);
}

std::vector<int> Table::getSizesTableQ() {
    std::vector<int> res;
    res.push_back(Table::tableQ.size());
    res.push_back(Actions::size);
    return res;
}

std::vector<int> Table::getSizesTableE() {
    std::vector<int> res;
    res.push_back(Table::tableE.size());
    res.push_back(Actions::size);
    return res;
}
