"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
class Trajectory {
    constructor(id, nDimensions) {
        this.m_trajectoryId = id;
        this.m_nDimensions = nDimensions;
        this.m_nPoints = 0;
        this.m_nPartitionPoints = 0;
        this.m_pointArray = new Array();
        this.m_partitionPointArray = new Array();
    }
    Trajectory() {
        this.m_trajectoryId = -1;
        this.m_nDimensions = 2;
        this.m_nPoints = 0;
        this.m_nPartitionPoints = 0;
        this.m_pointArray = new Array();
        this.m_partitionPointArray = new Array();
    }
    //two methods	
    addPointToArray(point) {
        this.m_pointArray.push(point);
        this.m_nPoints++;
    }
    addPartitionPointToArray(point) {
        this.m_partitionPointArray.push(point);
        this.m_nPartitionPoints++;
    }
    setM_trajectoryId(id) {
        this.m_trajectoryId = id;
    }
    getM_trajectoryId() {
        return this.m_trajectoryId;
    }
    getM_nDimensions() {
        return this.m_nDimensions;
    }
    setM_nDimensions(m_nDimensions) {
        this.m_nDimensions = m_nDimensions;
    }
    getM_nPoints() {
        return this.m_nPoints;
    }
    setM_nPoints(m_nPoints) {
        this.m_nPoints = m_nPoints;
    }
    getM_pointArray() {
        return this.m_pointArray;
    }
    setM_pointArray(m_pointArray) {
        this.m_pointArray = m_pointArray;
    }
    getM_nPartitionPoints() {
        return this.m_nPartitionPoints;
    }
    setM_nPartitionPoints(m_nPartitionPoints) {
        this.m_nPartitionPoints = m_nPartitionPoints;
    }
    getM_partitionPointArray() {
        return this.m_partitionPointArray;
    }
    setM_partitionPointArray(m_partitionPointArray) {
        this.m_partitionPointArray = m_partitionPointArray;
    }
}
exports.default = Trajectory;
//# sourceMappingURL=Trajectory.js.map