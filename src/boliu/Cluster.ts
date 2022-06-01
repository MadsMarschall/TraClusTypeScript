import CMDPoint from "./CMDPoint";

export default class Cluster {

	private m_clusterId: number;		// the identifier of this cluster
	private m_nDimensions: number;		// the dimensionality of this cluster
	private m_nTrajectories: number;	// the number of trajectories belonging to this cluster
	private m_nPoints: number;			// the number of points constituting a cluster 
	private m_pointArray: CMDPoint[]	// the array of the cluster points
	public constructor(id?: number, nDimensions?: number) {
		if (!id || !nDimensions) {
			this.m_clusterId = -1;
			this.m_nDimensions = 2;
			this.m_nTrajectories = 0;
			this.m_nPoints = 0;
			this.m_pointArray = new Array<CMDPoint>();
			return
        }
		this.m_clusterId = id;
		this.m_nDimensions = nDimensions;
		this.m_nTrajectories = 0;
		this.m_nPoints = 0;
		this.m_pointArray = new Array<CMDPoint>();
	}

	public setM_clusterId(m_clusterId: number): void {
		this.m_clusterId = m_clusterId;
	}

	public getM_clusterId(): number {
		return this.m_clusterId;
	}
	
	/**
	 * set m_nTrajectories --the number of trajectories belonging to this cluster
	 * @param density
	 */
	public setDensity(density:number): void {
		this.m_nTrajectories = density;
	}
	/**
	 * get the density -- the number of trajectories belonging to this cluster
	 * @return density number
	 */
	public getDensity():number {
		return this.m_nTrajectories;
	}
	
	public addPointToArray(point: CMDPoint):void {
		this.m_pointArray.push(point);
		this.m_nPoints++;				
	}
	
	public getM_PointArray(): CMDPoint[] {
		return this.m_pointArray;
	}

	public writeCluster(outfile: unknown): boolean {
		return true;	
	}
}
