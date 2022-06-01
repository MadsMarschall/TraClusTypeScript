import {ITraClusterDoc, logCaller} from "..";
import Cluster from "./Cluster";
import CMDPoint from "./CMDPoint";
import Trajectory from "./Trajectory";

export class LineSegmentId {

	trajectoryId: number | undefined
	order: number | undefined
}
export class CandidateClusterPoint {

	orderingValue: number | undefined;
	lineSegmentId: number | undefined;
	startPointFlag: boolean | undefined;
		
}
export class Parameter {
	epsParam: number | undefined;
	minLnsParam: number | undefined;
}

export class LineSegmentCluster {

	lineSegmentClusterId: number | undefined;
	nLineSegments: number | undefined;
	avgDirectionVector: CMDPoint | undefined;
	cosTheta: number | undefined;
	sinTheta: number | undefined;
	candidatePointList: Array<CandidateClusterPoint> = new Array<CandidateClusterPoint>();
	nClusterPoints: number | undefined;
	clusterPointArray: Array<CMDPoint> = new Array<CMDPoint>();
	nTrajectories: number | undefined;
	trajectoryIdList: Array<number> = new Array<number>();
	enabled: boolean | undefined;
}

export default class ClusterGen {
	//@ts-ignore
	public m_document: ITraClusterDoc;

	private m_epsParam: number | undefined;
	private m_minLnsParam: number | undefined;
	private m_nTotalLineSegments: number | undefined;
	private m_currComponentId: number | undefined;
	// the number of dense components discovered until now
	private m_componentIdArray: number[] = new Array<number>();
	// the list of line segment clusters
	private m_lineSegmentClusters: LineSegmentCluster[] | undefined;
	// programming trick: avoid frequent execution of the new and delete operations
	private m_startPoint1: CMDPoint | undefined;
	private m_endPoint1: CMDPoint | undefined;
	private m_startPoint2: CMDPoint | undefined;
	private m_endPoint2: CMDPoint | undefined;
	private m_vector1: CMDPoint | undefined ; //  = new CMDPoint(m_document.m_nDimensions);
	private m_vector2: CMDPoint | undefined ; // = new CMDPoint(m_document.m_nDimensions);;
	private m_projectionPoint: CMDPoint | undefined ; // = new CMDPoint( m_document.m_nDimensions);;
	private m_coefficient: number | undefined;
	
	private m_idArray: LineSegmentId[] = new Array<LineSegmentId>();
	private m_lineSegmentPointArray: CMDPoint[] = new Array<CMDPoint>();
	
	// used for performing the DBSCAN algorithm
	public  static UNCLASSIFIED = -2;
	public  static NOISE = -1;

	private static MIN_LINESEGMENT_LENGTH = 50.0;

	private static MDL_COST_ADWANTAGE = 25;
	private static INT_MAX = Number.MAX_VALUE;
	// used for InsertClusterPoint() and ReplaceClusterPoint() 
	public readonly PointLocation = {
		HEAD:13 , TAIL:32
	}
	

	

	// this default constructor should be never used	
	// use the following constructor instead
	public constructor(document?: ITraClusterDoc) {
		if (document) {
			this.m_document = <ITraClusterDoc>document;

			this.m_startPoint1 = new CMDPoint(this.m_document.m_nDimensions);
			this.m_startPoint2 = new CMDPoint(this.m_document.m_nDimensions);
			this.m_endPoint1 = new CMDPoint(this.m_document.m_nDimensions);
			this.m_endPoint2 = new CMDPoint(this.m_document.m_nDimensions);

			this.m_vector1 = new CMDPoint(this.m_document.m_nDimensions);
			this.m_vector2 = new CMDPoint(this.m_document.m_nDimensions);
			this.m_projectionPoint = new CMDPoint(this.m_document.m_nDimensions);

        }
		
		
		this.m_idArray = [];
		this.m_lineSegmentPointArray = [];
		
	}
	
	public constructCluster(): boolean {
		// this step consists of two sub-steps
		// notice that the result of the previous sub-step is used in the following sub-steps
		if (!this.constructLineSegmentCluster()) {
			return false;
		}
		if (!this.storeLineSegmentCluster()) {
			return false;
		}
		return true;
	}

	public partitionTrajectory(): boolean {
		
		for (let i = 0; i < this.m_document.m_trajectoryList.length; i++) {
			let pTrajectory: Trajectory = this.m_document.m_trajectoryList[i];
						
			this.findOptimalPartition(pTrajectory);	

			this.m_document.m_trajectoryList[0]= pTrajectory;
		}
		if (!this.storeClusterComponentIntoIndex()) {	
			return false;
		}
		return true;		
	}
	
	public performDBSCAN(eps,minLns)  {
		
		this.m_epsParam = eps;
		this.m_minLnsParam = minLns;

		this.m_currComponentId = 0;
				
		for (let i = 0; i < this.m_nTotalLineSegments; i++) {
			this.m_componentIdArray.push(ClusterGen.UNCLASSIFIED);
		}
		
		for (let i = 0; i < this.m_nTotalLineSegments; i++) {
			if (this.m_componentIdArray[i] == ClusterGen.UNCLASSIFIED && this.expandDenseComponent(i, this.m_currComponentId, eps, minLns)) {
				this.m_currComponentId++;
			}
		}
		return true;		
	}

	
	private storeClusterComponentIntoIndex() {
		let nDimensions = this.m_document.m_nDimensions;
		let startPoint: CMDPoint;
		let endPoint: CMDPoint;
		
		this.m_nTotalLineSegments = 0;
		for (let i = 0; i < this.m_document.m_trajectoryList.length; i++) {
			let pTrajectory: Trajectory = this.m_document.m_trajectoryList[i];
			
			for (let j =0; j<pTrajectory.getM_nPartitionPoints()-1; j++) {
				// convert an n-dimensional line segment into a 2n-dimensional point
				// i.e., the first n-dimension: the start point
				//       the last n-dimension: the end point
				startPoint = pTrajectory.getM_partitionPointArray()[j];
				endPoint = pTrajectory.getM_partitionPointArray()[j + 1];
				
				if (this.measureDistanceFromPointToPoint(startPoint, endPoint) <ClusterGen.MIN_LINESEGMENT_LENGTH) {
					continue;
				}
				this.m_nTotalLineSegments++;
				
				let lineSegmentPoint: CMDPoint = new CMDPoint(nDimensions * 2);
				for (let m = 0; m < nDimensions; m++) {
					lineSegmentPoint.setM_coordinate(m, startPoint.getM_coordinate(m));
					lineSegmentPoint.setM_coordinate(nDimensions+m, endPoint.getM_coordinate(m));
				}
				
				let id: LineSegmentId = new LineSegmentId();
				id.trajectoryId = pTrajectory.getM_trajectoryId();
				id.order = j;
				
				this.m_idArray.push(id);
				this.m_lineSegmentPointArray.push(lineSegmentPoint);
			}						
		}						
				
		return true;
	}

	private findOptimalPartition(pTrajectory: Trajectory) {
		
		let nPoints = pTrajectory.getM_nPoints();
		let startIndex = 0, length =0;
		let fullPartitionMDLCost, partialPartitionMDLCost;
		
		// add the start point of a trajectory
		let startP: CMDPoint = pTrajectory.getM_pointArray()[0];
		pTrajectory.addPartitionPointToArray(startP);
				
		for (;;) {
			fullPartitionMDLCost = partialPartitionMDLCost = 0;
			
			for (length = 1; startIndex + length < nPoints; length++) {
				// compute the total length of a trajectory
				fullPartitionMDLCost += this.computeModelCost(pTrajectory, startIndex+length - 1, startIndex + length);
				
				// compute the sum of (1) the length of a cluster component and 
				// 					 (2) the perpendicular and angle distances
				partialPartitionMDLCost = this.computeModelCost(pTrajectory, startIndex, startIndex + length) +
						this.computeEncodingCost(pTrajectory, startIndex, startIndex + length);
				
				if(fullPartitionMDLCost + ClusterGen.MDL_COST_ADWANTAGE < partialPartitionMDLCost) {
					
					pTrajectory.addPartitionPointToArray(pTrajectory.getM_pointArray()[startIndex+length-1]);
					startIndex = startIndex + length -1;
					length = 0;
					break;
				}
			}
			// if we reach at the end of a trajectory
			if (startIndex+length >= nPoints) {
				break;
			}
		}
		
		// add the end point of a trajectory
		pTrajectory.addPartitionPointToArray(pTrajectory.getM_pointArray()[nPoints-1]);
		
		return;
	}
	
	privateLOG2(x) {
		return Math.log(x) / Math.log(2);
	}
	
	private computeModelCost(pTrajectory,startPIndex,endPIndex) {

		let lineSegmentStart = pTrajectory.getM_pointArray()[(startPIndex)];
		let lineSegmentEnd = pTrajectory.getM_pointArray()[(endPIndex)];
		
		let distance = this.measureDistanceFromPointToPoint(lineSegmentStart, lineSegmentEnd);
		
		if (distance < 1.0) {
			distance = 1.0; 		// to take logarithm
		}

		return Math.ceil(Math.log2(distance));

	}

	private computeEncodingCost(pTrajectory: Trajectory,startPIndex:number,endPIndex:number) {

		let clusterComponentStart: CMDPoint;
		let clusterComponentEnd: CMDPoint;
		let lineSegmentStart: CMDPoint;
		let lineSegmentEnd: CMDPoint;
		let perpendicularDistance;
		let angleDistance;
		let encodingCost = 0;
		
		clusterComponentStart = pTrajectory.getM_pointArray()[startPIndex];
		clusterComponentEnd = pTrajectory.getM_pointArray()[endPIndex];
				
		for (let i = startPIndex; i < endPIndex; i++) {
			lineSegmentStart = pTrajectory.getM_pointArray()[i];
			lineSegmentEnd = pTrajectory.getM_pointArray()[i + 1];
			
			perpendicularDistance = this.measurePerpendicularDistance(clusterComponentStart,
					clusterComponentEnd, lineSegmentStart, lineSegmentEnd);
			angleDistance = this.measureAngleDisntance(clusterComponentStart,
					clusterComponentEnd, lineSegmentStart, lineSegmentEnd);
			
			if (perpendicularDistance < 1.0) perpendicularDistance = 1.0;	//  to take logarithm
			if (angleDistance < 1.0) angleDistance = 1.0;					//  to take logarithm

			encodingCost += (Math.ceil(Math.log2(perpendicularDistance)) + Math.ceil(Math.log2(angleDistance)));
		}
		return encodingCost;
		
	}
	private measurePerpendicularDistance(s1: CMDPoint, e1: CMDPoint, s2: CMDPoint, e2:CMDPoint) {

		//  we assume that the first line segment is longer than the second one
		let distance1;	//  the distance from a start point to the cluster component
		let distance2;	//  the distance from an end point to the cluster component

		distance1 = this.measureDistanceFromPointToLineSegment(s1, e1, s2);
		distance2 = this.measureDistanceFromPointToLineSegment(s1, e1, e2);

		//  if the first line segment is exactly the same as the second one, 
		//  the perpendicular distance should be zero
		if (distance1 == 0.0 && distance2 == 0.0) return 0.0;

		//  return (d1^2 + d2^2) / (d1 + d2) as the perpendicular distance
		return ((Math.pow(distance1, 2) + Math.pow(distance2, 2)) / (distance1 + distance2));

	}
	
	private measureDistanceFromPointToLineSegment(s: CMDPoint, e: CMDPoint, p: CMDPoint) {

		let nDimensions = p.getM_nDimensions();

		//  NOTE: the variables this.m_vector1 and m_vector2 are declared as member variables

		//  construct two vectors as follows
		//  1. the vector connecting the start point of the cluster component and a given point
		//  2. the vector representing the cluster component
		for (let i = 0; i <nDimensions; i++)
		{
			this.m_vector1.setM_coordinate(i, p.getM_coordinate(i) - s.getM_coordinate(i));
			this.m_vector2.setM_coordinate(i, e.getM_coordinate(i) - s.getM_coordinate(i));
		}

		//  a coefficient (0 <= b <= 1)
		this.m_coefficient = this.computeInnerProduct(this.m_vector1, this.m_vector2) / this.computeInnerProduct(this.m_vector2, this.m_vector2);

		//  the projection on the cluster component from a given point
		//  NOTE: the variable m_projectionPoint is declared as a member variable

		for (let i = 0; i <nDimensions; i++) {
			this.m_projectionPoint.setM_coordinate(i, s.getM_coordinate(i) + this.m_coefficient * this.m_vector2.getM_coordinate(i));
		}

		//  return the distance between the projection point and the given point
		return this.measureDistanceFromPointToPoint(p, this.m_projectionPoint);

	}

	private measureDistanceFromPointToPoint(point1: CMDPoint, point2: CMDPoint): number {
		let nDimensions = point1.getM_nDimensions();
		let squareSum = 0.0;
		
		for (let i = 0; i < nDimensions; i++) {
			squareSum += Math.pow((point2.getM_coordinate(i) - point1.getM_coordinate(i)), 2);
		}
		return Math.sqrt(squareSum);
		
	}
	
	private computeVectorLength(vector: CMDPoint) {
		
		let nDimensions = vector.getM_nDimensions();
		let squareSum = 0.0;
		
		for (let i = 0; i < nDimensions; i++) {
			squareSum += Math.pow(vector.getM_coordinate(i), 2);
		}
		
		return Math.sqrt(squareSum);		
	}
	
	private computeInnerProduct(vector1: CMDPoint, vector2: CMDPoint) {
		let nDimensions = vector1.getM_nDimensions();
		let innerProduct = 0.0;
		
		for (let i = 0; i < nDimensions; i++) {
			innerProduct += (vector1.getM_coordinate(i) * vector2.getM_coordinate(i));
		}
		
		return innerProduct;
	}
	private measureAngleDisntance(s1: CMDPoint, e1: CMDPoint, s2: CMDPoint, e2: CMDPoint) {
		
		let nDimensions = s1.getM_nDimensions();
		
		//  NOTE: the variables this.m_vector1 and this.m_vector2 are declared as member variables
		//  construct two vectors representing the cluster component and a line segment, respectively
		for (let i = 0; i < nDimensions; i++) {
			this.m_vector1.setM_coordinate(i, e1.getM_coordinate(i) - s1.getM_coordinate(i));
			this.m_vector2.setM_coordinate(i, e2.getM_coordinate(i) - s2.getM_coordinate(i));			
		}
		
		//  we assume that the first line segment is longer than the second one
		//  i.e., vectorLength1 >= vectorLength2
		let vectorLength1 = this.computeVectorLength(this.m_vector1);
		let vectorLength2 = this.computeVectorLength(this.m_vector2);

		//  if one of two vectors is a point, the angle distance becomes zero
		if (vectorLength1 == 0.0 || vectorLength2 == 0.0) return 0.0;
		
		//  compute the inner product of the two vectors
		let innerProduct = this.computeInnerProduct(this.m_vector1, this.m_vector2);

		//  compute the angle between two vectors by using the inner product
		let cosTheta = innerProduct / (vectorLength1 * vectorLength2);
		//  compensate the computation error (e.g., 1.00001)
		//  cos(theta) should be in the range [-1.0, 1.0]
		//  START ...
		if (cosTheta > 1.0) cosTheta = 1.0; 
		if (cosTheta < -1.0) cosTheta = -1.0;
		//  ... END
		let sinTheta = Math.sqrt(1 - Math.pow(cosTheta, 2));
		//  if 90 <= theta <= 270, the angle distance becomes the length of the line segment
		//  if (cosTheta < -1.0) sinTheta = 1.0;

		return (vectorLength2 * sinTheta);					
	}
	
	private expandDenseComponent( index,componentId,eps,minDensity) {
		
		let seeds = new Set<number>()
		let seedResult = new Set<number>();

		let currIndex: number;
		
		this.extractStartAndEndPoints(index, this.m_startPoint1, this.m_endPoint1);
		this.computeEPSNeighborhood(this.m_startPoint1, this.m_endPoint1, eps, seeds);
				
		if (seeds.size < minDensity) { //  not a core line segment

			this.m_componentIdArray[index] = ClusterGen.NOISE;
			return false;
		} 
		// else...
		for (let i = 0; i < seeds.size; i++) {
			this.m_componentIdArray[(Array.from(seeds)[i])] = componentId;
		}
		seeds.delete(index);
		while (!(seeds.size == 0)) {
			currIndex = Array.from(seeds)[0];				
			this.extractStartAndEndPoints(currIndex, this.m_startPoint1, this.m_endPoint1);
			this.computeEPSNeighborhood(this.m_startPoint1, this.m_endPoint1, eps, seedResult);
			
			if (seedResult.size >= minDensity) {
				for (let iter = 0; iter < seedResult.size; iter++) {
					if (this.m_componentIdArray[(Array.from(seedResult)[iter])] == ClusterGen.UNCLASSIFIED ||
						this.m_componentIdArray[(Array.from(seedResult)[iter])]  == ClusterGen.NOISE) {
						if (this.m_componentIdArray[(Array.from(seedResult)[iter])] == ClusterGen.UNCLASSIFIED) {
							seeds.add((Array.from(seedResult)[iter] ));
						}
						this.m_componentIdArray[(Array.from(seedResult)[iter])] = componentId;
					}
				}
			}
			
			seeds.delete(currIndex);
		}
		
		return true;	
	}
	
	private constructLineSegmentCluster() {
		let nDimensions = this.m_document.m_nDimensions;
		this.m_lineSegmentClusters = Array(this.m_currComponentId);//(this.m_currComponentId);
				
		//  initialize the list of line segment clusters
		//  START ...
		for (let i = 0; i < this.m_currComponentId; i++) {	
			this.m_lineSegmentClusters[i] = new LineSegmentCluster();
			this.m_lineSegmentClusters[i].avgDirectionVector = new CMDPoint(nDimensions);
			this.m_lineSegmentClusters[i].lineSegmentClusterId = i;
			this.m_lineSegmentClusters[i].nLineSegments = 0;
			this.m_lineSegmentClusters[i].nClusterPoints = 0;
			this.m_lineSegmentClusters[i].nTrajectories = 0;
			this.m_lineSegmentClusters[i].enabled = false;
		}
		//  ... END

		//  accumulate the direction vector of a line segment
		for (let i = 0; i < this.m_nTotalLineSegments; i++) {
			let componentId = this.m_componentIdArray[i];
			if (componentId >= 0) {
				for (let j = 0; j < nDimensions; j++) {
					let difference = this.m_lineSegmentPointArray[(i)].getM_coordinate(nDimensions + j)
							- this.m_lineSegmentPointArray[i].getM_coordinate(j);
					let currSum = this.m_lineSegmentClusters[componentId].avgDirectionVector.getM_coordinate(j)
							+ difference;
					this.m_lineSegmentClusters[componentId].avgDirectionVector.setM_coordinate(j, currSum);
				}
				this.m_lineSegmentClusters[componentId].nLineSegments++;
			}
		}

		//  compute the average direction vector of a line segment cluster
		//  START ...
		let vectorLength1, vectorLength2, innerProduct;
		let cosTheta, sinTheta;

		this.m_vector2.setM_coordinate(0, 1.0);
		this.m_vector2.setM_coordinate(1, 0.0);

		for (let i = 0; i < this.m_currComponentId; i++) {
			let clusterEntry: LineSegmentCluster =this.m_lineSegmentClusters[i];

			for (let j = 0; j < nDimensions; j++) {
				clusterEntry.avgDirectionVector.setM_coordinate(j, clusterEntry.avgDirectionVector.getM_coordinate(j) / clusterEntry.nLineSegments);
			}
			vectorLength1 = this.computeVectorLength(clusterEntry.avgDirectionVector);
			vectorLength2 = 1.0;

			innerProduct = this.computeInnerProduct(clusterEntry.avgDirectionVector, this.m_vector2);
			cosTheta = innerProduct / (vectorLength1 * vectorLength2);
			if (cosTheta > 1.0) cosTheta = 1.0; 
			if (cosTheta < -1.0) cosTheta = -1.0;
			sinTheta = Math.sqrt(1 - Math.pow(cosTheta, 2));

			if (clusterEntry.avgDirectionVector.getM_coordinate(1) < 0) {
				sinTheta = -sinTheta;
			}

			clusterEntry.cosTheta = cosTheta;
			clusterEntry.sinTheta = sinTheta;
			
		}
		//  ... END

		//  summarize the information about line segment clusters
		//  the structure for summarization is as follows
		//  [lineSegmentClusterId, nClusterPoints, clusterPointArray, nTrajectories, { trajectoryId, ... }]
		for (let i = 0; i < this.m_nTotalLineSegments; i++) {
			if (this.m_componentIdArray[i] >= 0)		//  if the componentId < 0, it is a noise
				this.RegisterAndUpdateLineSegmentCluster(this.m_componentIdArray[i], i);
		}

		let trajectories: Set<Number>  = new Set<Number>();
		for (let i = 0; i < this.m_currComponentId; i++) {
			let clusterEntry: LineSegmentCluster = (this.m_lineSegmentClusters[i]);

			//  a line segment cluster must have trajectories more than the minimum threshold
			if (clusterEntry.nTrajectories >= this.m_minLnsParam) {
				clusterEntry.enabled = true;
				//this.m_lineSegmentClusters[i].enabled = true;
				//  DEBUG: count the number of trajectories that belong to clusters
				for (let j = 0; j < clusterEntry.trajectoryIdList.length; j++) {
					trajectories.add(clusterEntry.trajectoryIdList[j]);
				}

				this.computeRepresentativeLines(clusterEntry);
				// computeRepresentativeLines(this.m_lineSegmentClusters[i]);
			} else {
				clusterEntry.candidatePointList = [];
				clusterEntry.clusterPointArray = [];
				clusterEntry.trajectoryIdList = [];			
			}			
			
		}
		//  DEBUG: compute the ratio of trajectories that belong to clusters
		this.m_document.m_clusterRatio = trajectories.size / this.m_document.m_nTrajectories;

		return true;	
	}
	
	
	private computeRepresentativeLines(clusterEntry: LineSegmentCluster) {

		let lineSegments: Set<number> = new Set<number>();
		let insertionList: Set<number> = new Set<number>();
		let deletionList: Set<number> = new Set<number>();

		let iter: number = 0;
		let candidatePoint: CandidateClusterPoint, nextCandidatePoint: CandidateClusterPoint;
		let prevOrderingValue: number = 0.0;

		let nClusterPoints = 0;
		lineSegments.clear();

		//  sweep the line segments in a line segment cluster

		while (iter != (clusterEntry.candidatePointList.length - 1) && clusterEntry.candidatePointList.length > 0) {
			insertionList.clear()
			deletionList.clear()
			
			do {
				candidatePoint = clusterEntry.candidatePointList[(iter)];
				iter++;
				//  check whether this line segment has begun or not
				if (!lineSegments.has(candidatePoint.lineSegmentId)) {
				// iter1 = lineSegments.find(candidatePoint.lineSegmentId);
				// if (iter1 == lineSegments.end())	{				//  if there is no matched element,
					insertionList.add(candidatePoint.lineSegmentId);		//  this line segment begins at this point
					lineSegments.add(candidatePoint.lineSegmentId);
				} else {						//  if there is a matched element,
					deletionList.add(candidatePoint.lineSegmentId);		//  this line segment ends at this point
				}
				//  check whether the next line segment begins or ends at the same point
				if (iter != (clusterEntry.candidatePointList.length - 1)) {
					nextCandidatePoint = clusterEntry.candidatePointList[iter];
				} else {
					break;
				}
			} while (candidatePoint.orderingValue == nextCandidatePoint.orderingValue);
			
			//  check if a line segment is connected to another line segment in the same trajectory
			//  if so, delete one of the line segments to remove duplicates
			// for (iter2 = insertionList.begin(); iter2 != insertionList.end(); iter2++)
			for (let iter2 = 0; iter2 < insertionList.size; iter2++) {
				for (let iter3 = 0; iter3 < deletionList.size; iter3++) {
					let a = (Array.from(insertionList)[iter2]);
					let b = (Array.from(deletionList)[iter3]);
					if (a == b) {
						lineSegments.delete((Array.from(deletionList)[iter3]));
						deletionList.delete((Array.from(deletionList)[iter3]));
						break;
					}
				}
				
				for (let iter3 = 0; iter3 < deletionList.size; iter3++) {
					if (this.m_idArray[(Array.from(insertionList)[iter2])].trajectoryId
							== this.m_idArray[(Array.from(deletionList)[iter3])].trajectoryId) {
						lineSegments.delete((Array.from(deletionList)[iter3]));
						deletionList.delete((Array.from(deletionList)[iter3]));	
						break;
					}
				}
			}
			
			// if the current density exceeds a given threshold
			if ((lineSegments.size) >= this.m_minLnsParam) {
				if (Math.abs(candidatePoint.orderingValue - prevOrderingValue) > (ClusterGen.MIN_LINESEGMENT_LENGTH / 1.414)) {
					this.computeAndRegisterClusterPoint(clusterEntry, candidatePoint.orderingValue, lineSegments);
					prevOrderingValue = candidatePoint.orderingValue;
					nClusterPoints++;
				}
			}

			//  delete the line segment that is not connected to another line segment
			for (let iter3=0; iter3<deletionList.size; iter3 ++) {	
				lineSegments.delete((Array.from(deletionList)[iter3]));
			}
		}

		if (nClusterPoints >= 2) {
			clusterEntry.nClusterPoints = nClusterPoints;
		} else {
			//  there is no representative trend in this line segment cluster
			clusterEntry.enabled = false;
			clusterEntry.candidatePointList = [];
			clusterEntry.clusterPointArray = [];
			clusterEntry.trajectoryIdList = [];
		}	
		return;	
	}
	
	private computeAndRegisterClusterPoint(
		clusterEntry: LineSegmentCluster,
			currValue,
		lineSegments: Set<number>) {
		let nDimensions = this.m_document.m_nDimensions;
		let nLineSegmentsInSet = (lineSegments.size);
		let clusterPoint: CMDPoint = new CMDPoint(nDimensions);
		let sweepPoint: CMDPoint = new CMDPoint(nDimensions);

		for (let iter = 0; iter < lineSegments.size; iter++) {
			// get the sweep point of each line segment
			// this point is parallel to the current value of the sweeping direction
			this.getSweepPointOfLineSegment(clusterEntry, currValue,
				(Array.from(lineSegments)[iter]), sweepPoint);
			for (let i = 0; i < nDimensions; i++) {
				clusterPoint.setM_coordinate(i, clusterPoint.getM_coordinate(i) +
						(sweepPoint.getM_coordinate(i) / nLineSegmentsInSet));
			}
		}
		
		// NOTE: this program code works only for the 2-dimensional data
		let origX, origY;
		origX = this.GET_X_REV_ROTATION(clusterPoint.getM_coordinate(0), clusterPoint.getM_coordinate(1), clusterEntry.cosTheta, clusterEntry.sinTheta);
		origY = this.GET_Y_REV_ROTATION(clusterPoint.getM_coordinate(0), clusterPoint.getM_coordinate(1), clusterEntry.cosTheta, clusterEntry.sinTheta);
		clusterPoint.setM_coordinate(0, origX);
		clusterPoint.setM_coordinate(1, origY);

		// register the obtained cluster point (i.e., the average of all the sweep points)
		clusterEntry.clusterPointArray.push(clusterPoint);

		return;		
	}
	
	private getSweepPointOfLineSegment(clusterEntry: LineSegmentCluster,
		currValue, lineSegmentId, sweepPoint: CMDPoint) {

		let lineSegmentPoint: CMDPoint = this.m_lineSegmentPointArray[(lineSegmentId)];		//  2n-dimensional point
		let coefficient;
		
		//  NOTE: this program code works only for the 2-dimensional data
		let newStartX, newEndX, newStartY, newEndY;
		newStartX = this.GET_X_ROTATION(lineSegmentPoint.getM_coordinate(0), lineSegmentPoint.getM_coordinate(1), clusterEntry.cosTheta, clusterEntry.sinTheta);
		newEndX   = this.GET_X_ROTATION(lineSegmentPoint.getM_coordinate(2), lineSegmentPoint.getM_coordinate(3), clusterEntry.cosTheta, clusterEntry.sinTheta);
		newStartY = this.GET_Y_ROTATION(lineSegmentPoint.getM_coordinate(0), lineSegmentPoint.getM_coordinate(1), clusterEntry.cosTheta, clusterEntry.sinTheta);
		newEndY   = this.GET_Y_ROTATION(lineSegmentPoint.getM_coordinate(2), lineSegmentPoint.getM_coordinate(3), clusterEntry.cosTheta, clusterEntry.sinTheta);

		coefficient = (currValue - newStartX) / (newEndX - newStartX);
		sweepPoint.setM_coordinate(0, currValue);
		sweepPoint.setM_coordinate(1, newStartY + coefficient * (newEndY - newStartY));

		return;				
	}
	
	
	private GET_X_ROTATION(_x,_y,_cos,_sin) {
		return ((_x)*(_cos) + (_y)*(_sin));
	}
	private GET_Y_ROTATION(_x,_y,_cos,_sin) {
		return  (-(_x)*(_sin) + (_y)*(_cos));
	}
	private GET_X_REV_ROTATION(_x,_y,_cos,_sin) {
		return ((_x)*(_cos) - (_y)*(_sin));
	}
	private GET_Y_REV_ROTATION(_x,_y,_cos,_sin) {
		return ((_x)*(_sin) + (_y)*(_cos));
	}

	private RegisterAndUpdateLineSegmentCluster(componentId: number, lineSegmentId: number) {
		let clusterEntry: LineSegmentCluster = this.m_lineSegmentClusters[componentId];

		//  the start and end values of the first dimension (e.g., the x value in the 2-dimension)
		//  NOTE: this program code works only for the 2-dimensional data

		let aLineSegment: CMDPoint = this.m_lineSegmentPointArray[(lineSegmentId)];
		let orderingValue1 = this.GET_X_ROTATION(aLineSegment.getM_coordinate(0),
				aLineSegment.getM_coordinate(1), clusterEntry.cosTheta, clusterEntry.sinTheta); 
		let orderingValue2 = this.GET_X_ROTATION(aLineSegment.getM_coordinate(2), 
				aLineSegment.getM_coordinate(3), clusterEntry.cosTheta, clusterEntry.sinTheta); 

		let existingCandidatePoint: CandidateClusterPoint, newCandidatePoint1: CandidateClusterPoint, newCandidatePoint2: CandidateClusterPoint;
		let i, j;
		//  sort the line segment points by the coordinate of the first dimension
		//  simply use the insertion sort algorithm
		//  START ...
		let iter1 = 0;
		for (i = 0; i < clusterEntry.candidatePointList.length; i++) {
			existingCandidatePoint = clusterEntry.candidatePointList[iter1];
			if (existingCandidatePoint.orderingValue >= orderingValue1) {
				break;
			}
			iter1++;
		}
		newCandidatePoint1 = new CandidateClusterPoint();
		
		newCandidatePoint1.orderingValue = orderingValue1;
		newCandidatePoint1.lineSegmentId = lineSegmentId;
		newCandidatePoint1.startPointFlag = true;
		if (i == 0) {
			clusterEntry.candidatePointList.splice(0, 0, newCandidatePoint1);
		} else if (i >= clusterEntry.candidatePointList.length) {
			clusterEntry.candidatePointList.push(newCandidatePoint1);
		} else {
			clusterEntry.candidatePointList.splice(iter1,0, newCandidatePoint1);
		}
		let iter2 = 0;
		for (j = 0; j < clusterEntry.candidatePointList.length; j++) {
			existingCandidatePoint = clusterEntry.candidatePointList[(iter2)];
			if (existingCandidatePoint.orderingValue >= orderingValue2) {
				break;
			}
			iter2++;
		}

		newCandidatePoint2 = new CandidateClusterPoint();
		newCandidatePoint2.orderingValue = orderingValue2;
		newCandidatePoint2.lineSegmentId = lineSegmentId;
		newCandidatePoint2.startPointFlag = false;

		if (j == 0) {
			clusterEntry.candidatePointList.splice(0, 0, newCandidatePoint2);
		} else if (j >= clusterEntry.candidatePointList.length) {
			clusterEntry.candidatePointList.push(newCandidatePoint2);
		} else {
			clusterEntry.candidatePointList.splice(iter2,0, newCandidatePoint2);
		}
		//  ... END
		
		let  trajectoryId = this.m_idArray[lineSegmentId].trajectoryId;

		//  store the identifier of the trajectories that belong to this line segment cluster
		if (!clusterEntry.trajectoryIdList.includes(trajectoryId)) {
			clusterEntry.trajectoryIdList.push(trajectoryId);
			clusterEntry.nTrajectories++;
		}
		return;
	}
	private computeEPSNeighborhood(startPoint: CMDPoint, endPoint: CMDPoint, eps, result: Set<number> ) {
		result.clear();
		for (let j = 0; j < this.m_nTotalLineSegments; j++) {
			this.extractStartAndEndPoints(j, this.m_startPoint2, this.m_endPoint2);
			let distance = this.computeDistanceBetweenTwoLineSegments(startPoint, endPoint, this.m_startPoint2, this.m_endPoint2);
			//  if the distance is below the threshold, this line segment belongs to the eps-neighborhood
			if (distance <= eps) result.add(j);
		}
		return;
	}
	private computeDistanceBetweenTwoLineSegments(startPoint1: CMDPoint,
		endPoint1: CMDPoint, startPoint2: CMDPoint, endPoint2: CMDPoint) {
		let perpendicularDistance = 0;
		let parallelDistance =0;
		let angleDistance =0;

		return this.subComputeDistanceBetweenTwoLineSegments(startPoint1, endPoint1, startPoint2, endPoint2, perpendicularDistance, parallelDistance, angleDistance);		
	}
	
	
	private storeLineSegmentCluster(): boolean {
		
		let currClusterId = 0;
		
		for (let i = 0; i < this.m_currComponentId; i++) {
			
			if (!this.m_lineSegmentClusters[i].enabled) {
				continue;
			}
			
			//  store the clusters finally identified
			//  START ...
			let pClusterItem: Cluster = new Cluster(currClusterId, this.m_document.m_nDimensions);

			for (let j = 0; j < this.m_lineSegmentClusters[i].nClusterPoints; j++) {
				pClusterItem.addPointToArray(this.m_lineSegmentClusters[i].clusterPointArray[j]);
			}
			
			pClusterItem.setDensity(this.m_lineSegmentClusters[i].nTrajectories);

			this.m_document.m_clusterList.push(pClusterItem);

			currClusterId++;	//  increase the number of final clusters
			//  ... END
		}
		
		this.m_document.m_nClusters = currClusterId;
		return true;
	}
	
	
	private subComputeDistanceBetweenTwoLineSegments(startPoint1: CMDPoint,
		endPoint1: CMDPoint, startPoint2: CMDPoint, endPoint2: CMDPoint,
			perpendicularDistance,parallelDistance,
			angleDistance) {
		
		let perDistance1, perDistance2;
		let parDistance1, parDistance2;
		let length1, length2;		

		//  the length of the first line segment
		length1 = this.measureDistanceFromPointToPoint(startPoint1, endPoint1);
		//  the length of the second line segment
		length2 = this.measureDistanceFromPointToPoint(startPoint2, endPoint2);

		//  compute the perpendicular distance and the parallel distance
		//  START ...
		if (length1 > length2) {
			perDistance1 = this.measureDistanceFromPointToLineSegment(startPoint1, endPoint1, startPoint2);
			if (this.m_coefficient < 0.5) parDistance1 = this.measureDistanceFromPointToPoint(startPoint1, this.m_projectionPoint);
			else parDistance1 = this.measureDistanceFromPointToPoint(endPoint1, this.m_projectionPoint);

			perDistance2 = this.measureDistanceFromPointToLineSegment(startPoint1, endPoint1, endPoint2);
			if (this.m_coefficient < 0.5) parDistance2 = this.measureDistanceFromPointToPoint(startPoint1, this.m_projectionPoint);
			else parDistance2 = this.measureDistanceFromPointToPoint(endPoint1, this.m_projectionPoint);
		} else {
			perDistance1 = this.measureDistanceFromPointToLineSegment(startPoint2, endPoint2, startPoint1);
			if (this.m_coefficient < 0.5) parDistance1 = this.measureDistanceFromPointToPoint(startPoint2, this.m_projectionPoint);
			else parDistance1 = this.measureDistanceFromPointToPoint(endPoint2, this.m_projectionPoint);

			perDistance2 = this.measureDistanceFromPointToLineSegment(startPoint2, endPoint2, endPoint1);
			if (this.m_coefficient < 0.5) parDistance2 = this.measureDistanceFromPointToPoint(startPoint2, this.m_projectionPoint);
			else parDistance2 = this.measureDistanceFromPointToPoint(endPoint2, this.m_projectionPoint);			
			
		}

		//  compute the perpendicular distance; take (d1^2 + d2^2) / (d1 + d2)
		if (!(perDistance1 == 0.0 && perDistance2 == 0.0)) {
			perpendicularDistance = ((Math.pow(perDistance1, 2) + Math.pow(perDistance2, 2)) / (perDistance1 + perDistance2));
		} else {
			perpendicularDistance = 0.0;
		}
		//  compute the parallel distance; take the minimum
		parallelDistance = (parDistance1 < parDistance2) ? parDistance1 : parDistance2;
		//  ... END
		
		//  compute the angle distance
		//  START ...
		//  MeasureAngleDisntance() assumes that the first line segment is longer than the second one
		if (length1 > length2) {
			angleDistance = this.measureAngleDisntance(startPoint1, endPoint1, startPoint2, endPoint2);
		} else {
			angleDistance = this.measureAngleDisntance(startPoint2, endPoint2, startPoint1, endPoint1);
		}
		//  ... END

		return (perpendicularDistance + parallelDistance + angleDistance);		
		
		
	}
	private extractStartAndEndPoints(index, startPoint: CMDPoint, endPoint: CMDPoint) {//  for speedup
		//  compose the start and end points of the line segment
		for (let i = 0; i < this.m_document.m_nDimensions; i++) {
			startPoint.setM_coordinate(i, this.m_lineSegmentPointArray[index].getM_coordinate(i));
			endPoint.setM_coordinate(i, this.m_lineSegmentPointArray[index].getM_coordinate(this.m_document.m_nDimensions + i));;
		}
	}

	public estimateParameterValue(p: Parameter) {

		let entropy, minEntropy = ClusterGen.INT_MAX;
		let eps, minEps = ClusterGen.INT_MAX;
		let totalSize, minTotalSize = ClusterGen.INT_MAX;
		let seeds = new Set<number>();

		let EpsNeighborhoodSize: number[] = Array(this.m_nTotalLineSegments);
		
		for (let eps = 20.0; eps <= 40.0; eps += 1.0) {
			entropy = 0.0;
			totalSize = 0;
			seeds.clear();
			for (let i = 0; i < this.m_nTotalLineSegments; i++) {
				this.extractStartAndEndPoints(i, this.m_startPoint1, this.m_endPoint1);
				this.computeEPSNeighborhood(this.m_startPoint1, this.m_endPoint1, eps, seeds);
				EpsNeighborhoodSize[i] = seeds.size;
				totalSize += seeds.size;
				seeds.clear();
			}
			for (let i = 0; i < this.m_nTotalLineSegments; i++) {
				entropy += (EpsNeighborhoodSize[i] / totalSize) * Math.log2(EpsNeighborhoodSize[i] / totalSize);
			}	
			entropy = -entropy;

			if (entropy < minEntropy)
			{
				minEntropy = entropy;
				minTotalSize = totalSize;
				minEps = eps;
			}
		}	
		// setup output arguments
		p.epsParam = minEps;
		p.minLnsParam = Math.ceil(minTotalSize / this.m_nTotalLineSegments);
		return true;		
	}
	
}
