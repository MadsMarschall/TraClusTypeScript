import CMDPoint from "./CMDPoint";
export default class Trajectory {
    private m_trajectoryId;
    private m_nDimensions;
    private m_nPoints;
    private m_pointArray;
    private m_nPartitionPoints;
    private m_partitionPointArray;
    Trajectory(): void;
    constructor(id: number, nDimensions: number);
    addPointToArray(point: CMDPoint): void;
    addPartitionPointToArray(point: CMDPoint): void;
    setM_trajectoryId(id: number): void;
    getM_trajectoryId(): number;
    getM_nDimensions(): number;
    setM_nDimensions(m_nDimensions: number): void;
    getM_nPoints(): number;
    setM_nPoints(m_nPoints: number): void;
    getM_pointArray(): Array<CMDPoint>;
    setM_pointArray(m_pointArray: Array<CMDPoint>): void;
    getM_nPartitionPoints(): number;
    setM_nPartitionPoints(m_nPartitionPoints: number): void;
    getM_partitionPointArray(): Array<CMDPoint>;
    setM_partitionPointArray(m_partitionPointArray: Array<CMDPoint>): void;
}
