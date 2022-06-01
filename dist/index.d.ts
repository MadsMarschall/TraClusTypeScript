import Cluster from "./boliu/Cluster";
import { Parameter } from "./boliu/ClusterGen";
import Trajectory from "./boliu/Trajectory";
export interface ITraClusterDoc {
    m_nDimensions: number;
    m_nTrajectories: number;
    m_nClusters: number;
    m_clusterRatio: number;
    m_maxNPoints: number;
    m_trajectoryList: Array<Trajectory>;
    m_clusterList: Array<Cluster>;
    onOpenDocument(inputFileName: number): boolean;
    onClusterGenerate(clusterFileName: string, epsParam: number, minLnsParam: number): boolean;
    onEstimateParameter(): Parameter;
}
