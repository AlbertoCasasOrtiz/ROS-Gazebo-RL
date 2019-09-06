//
// Created by alberto on 6/09/19.
//

#include "QTable.h"

void QTable::initializeTableQ() {
    // TODO
}

void QTable::initializeTableE() {
    // TODO
}

void QTable::updateValueQ(int x, int y, int a, float value) {
    QTable::tableQ.at(x).at(y).at(a) = QTable::tableQ.at(x).at(y).at(a) + value;
}

void QTable::updateValueE(int x, int y, int a, float value) {
    QTable::tableE.at(x).at(y).at(a) = QTable::tableE.at(x).at(y).at(a) + value;
}

float QTable::getValueQ(int x, int y, int a) {
    return QTable::tableQ.at(x).at(y).at(a);
}

float QTable::getValueE(int x, int y, int a) {
    return QTable::tableE.at(x).at(y).at(a);
}

std::vector<int> QTable::getSizesTableQ() {
    // TODO ver como mejorar
    std::vector<int> res;
    res.push_back(QTable::tableQ.size());
    res.push_back(QTable::tableQ.at(0).size());
    res.push_back(QTable::tableQ.at(0).at(0).size());
    return res;
}

std::vector<int> QTable::getSizesTableE() {
    // TODO ver como mejorar
    std::vector<int> res;
    res.push_back(QTable::tableE.size());
    res.push_back(QTable::tableE.at(0).size());
    res.push_back(QTable::tableE.at(0).at(0).size());
    return res;
}
