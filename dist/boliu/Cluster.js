"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
class Cluster {
    constructor(id, nDimensions) {
        if (!id || !nDimensions) {
            this.m_clusterId = -1;
            this.m_nDimensions = 2;
            this.m_nTrajectories = 0;
            this.m_nPoints = 0;
            this.m_pointArray = new Array();
            return;
        }
        this.m_clusterId = id;
        this.m_nDimensions = nDimensions;
        this.m_nTrajectories = 0;
        this.m_nPoints = 0;
        this.m_pointArray = new Array();
    }
    setM_clusterId(m_clusterId) {
        this.m_clusterId = m_clusterId;
    }
    getM_clusterId() {
        return this.m_clusterId;
    }
    /**
     * set m_nTrajectories --the number of trajectories belonging to this cluster
     * @param density
     */
    setDensity(density) {
        this.m_nTrajectories = density;
    }
    /**
     * get the density -- the number of trajectories belonging to this cluster
     * @return density number
     */
    getDensity() {
        return this.m_nTrajectories;
    }
    addPointToArray(point) {
        this.m_pointArray.push(point);
        this.m_nPoints++;
    }
    getM_PointArray() {
        return this.m_pointArray;
    }
    writeCluster(outfile) {
        return true;
    }
}
exports.default = Cluster;
//# sourceMappingURL=Cluster.js.map