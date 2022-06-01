
import CMDPoint from "./CMDPoint";

export default class Trajectory {
	
	private m_trajectoryId:number;	// the identifier of this trajectory
	private m_nDimensions: number; // the dimensionality of this trajectory
	private m_nPoints: number; // the number of points constituting a trajectory
	private m_pointArray: CMDPoint[]; // the array of the trajectory points
	private m_nPartitionPoints: number; // the number of partition points in a trajectory
	private m_partitionPointArray: CMDPoint[]; // the array of the partition points
	
	
	public Trajectory() {	
		this.m_trajectoryId = -1;
		this.m_nDimensions = 2;
		this.m_nPoints = 0;
		this.m_nPartitionPoints = 0;
		this.m_pointArray = new Array<CMDPoint>();
		this.m_partitionPointArray = new Array<CMDPoint>();
		
	}
	
	public constructor(id:number, nDimensions:number) {
		this.m_trajectoryId = id;
		this.m_nDimensions = nDimensions;
		this.m_nPoints = 0;
		this.m_nPartitionPoints = 0;
		this.m_pointArray = new Array<CMDPoint>();
		this.m_partitionPointArray = new Array<CMDPoint>();
	}
	//two methods	
	public addPointToArray(point: CMDPoint):void {
		this.m_pointArray.push(point);
		this.m_nPoints++;
	}

	public addPartitionPointToArray(point: CMDPoint): void {
		this.m_partitionPointArray.push(point);		
		this.m_nPartitionPoints++;
	}
	
	public setM_trajectoryId(id:number):void {
		this.m_trajectoryId = id;
	}
	
	public getM_trajectoryId() {
		return this.m_trajectoryId;
	}						
	
	public getM_nDimensions() {
		return this.m_nDimensions;
	}

	public setM_nDimensions(m_nDimensions:number) {
		this.m_nDimensions = m_nDimensions;
	}

	public getM_nPoints():number {
		return this.m_nPoints;
	}

	public setM_nPoints(m_nPoints:number) {
		this.m_nPoints = m_nPoints;
	}

	public getM_pointArray(): Array<CMDPoint> {
		return this.m_pointArray;
	}

	public setM_pointArray(m_pointArray: Array<CMDPoint>):void {
		this.m_pointArray = m_pointArray;
	}

	public getM_nPartitionPoints(): number {
		return this.m_nPartitionPoints;
	}

	public setM_nPartitionPoints(m_nPartitionPoints:number) {
		this.m_nPartitionPoints = m_nPartitionPoints;
	}

	public getM_partitionPointArray(): Array<CMDPoint> {
		return this.m_partitionPointArray;
	}

	public setM_partitionPointArray(m_partitionPointArray: Array<CMDPoint>) {
		this.m_partitionPointArray = m_partitionPointArray;
	}


 
}
