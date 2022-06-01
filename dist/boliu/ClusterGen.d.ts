import { ITraClusterDoc } from "..";
import CMDPoint from "./CMDPoint";
export declare class LineSegmentId {
    trajectoryId: number | undefined;
    order: number | undefined;
}
export declare class CandidateClusterPoint {
    orderingValue: number | undefined;
    lineSegmentId: number | undefined;
    startPointFlag: boolean | undefined;
}
export declare class Parameter {
    epsParam: number | undefined;
    minLnsParam: number | undefined;
}
export declare class LineSegmentCluster {
    lineSegmentClusterId: number | undefined;
    nLineSegments: number | undefined;
    avgDirectionVector: CMDPoint | undefined;
    cosTheta: number | undefined;
    sinTheta: number | undefined;
    candidatePointList: Array<CandidateClusterPoint>;
    nClusterPoints: number | undefined;
    clusterPointArray: Array<CMDPoint>;
    nTrajectories: number | undefined;
    trajectoryIdList: Array<number>;
    enabled: boolean | undefined;
}
export default class ClusterGen {
    m_document: ITraClusterDoc;
    private m_epsParam;
    private m_minLnsParam;
    private m_nTotalLineSegments;
    private m_currComponentId;
    private m_componentIdArray;
    private m_lineSegmentClusters;
    private m_startPoint1;
    private m_endPoint1;
    private m_startPoint2;
    private m_endPoint2;
    private m_vector1;
    private m_vector2;
    private m_projectionPoint;
    private m_coefficient;
    private m_idArray;
    private m_lineSegmentPointArray;
    static UNCLASSIFIED: number;
    static NOISE: number;
    private static MIN_LINESEGMENT_LENGTH;
    private static MDL_COST_ADWANTAGE;
    private static INT_MAX;
    readonly PointLocation: {
        HEAD: number;
        TAIL: number;
    };
    constructor(document?: ITraClusterDoc);
    constructCluster(): boolean;
    partitionTrajectory(): boolean;
    performDBSCAN(eps: any, minLns: any): boolean;
    private storeClusterComponentIntoIndex;
    private findOptimalPartition;
    privateLOG2(x: any): number;
    private computeModelCost;
    private computeEncodingCost;
    private measurePerpendicularDistance;
    private measureDistanceFromPointToLineSegment;
    private measureDistanceFromPointToPoint;
    private computeVectorLength;
    private computeInnerProduct;
    private measureAngleDisntance;
    private expandDenseComponent;
    private constructLineSegmentCluster;
    private computeRepresentativeLines;
    private computeAndRegisterClusterPoint;
    private getSweepPointOfLineSegment;
    private GET_X_ROTATION;
    private GET_Y_ROTATION;
    private GET_X_REV_ROTATION;
    private GET_Y_REV_ROTATION;
    private RegisterAndUpdateLineSegmentCluster;
    private computeEPSNeighborhood;
    private computeDistanceBetweenTwoLineSegments;
    private storeLineSegmentCluster;
    private subComputeDistanceBetweenTwoLineSegments;
    private extractStartAndEndPoints;
    estimateParameterValue(p: Parameter): boolean;
}
