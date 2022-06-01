import CMDPoint from "./CMDPoint";
export default class Cluster {
    private m_clusterId;
    private m_nDimensions;
    private m_nTrajectories;
    private m_nPoints;
    private m_pointArray;
    constructor(id?: number, nDimensions?: number);
    setM_clusterId(m_clusterId: number): void;
    getM_clusterId(): number;
    /**
     * set m_nTrajectories --the number of trajectories belonging to this cluster
     * @param density
     */
    setDensity(density: number): void;
    /**
     * get the density -- the number of trajectories belonging to this cluster
     * @return density number
     */
    getDensity(): number;
    addPointToArray(point: CMDPoint): void;
    getM_PointArray(): CMDPoint[];
    writeCluster(outfile: unknown): boolean;
}
