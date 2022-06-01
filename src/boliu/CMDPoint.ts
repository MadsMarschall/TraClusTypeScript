

export default class CMDPoint {
	
	
	private m_nDimensions:number; 	// the number of dimensions of a point
	public m_coordinate:number[] = new Array();	    // the coordinate of a point
	
	//default constructor which shall be never used, we can use the following constructor instead

	
	public constructor(nDimensions?: number) {
		if(!nDimensions) {
			this.m_nDimensions = 2;
			this.m_coordinate.length = this.m_nDimensions;
			this.m_coordinate[0] = 0;
			this.m_coordinate[1] = 0;
		}
		this.m_nDimensions = nDimensions;
		this.m_coordinate.length = this.m_nDimensions;
		for( let i=0; i < this.m_nDimensions; i++ ) {
			this.m_coordinate[i] = 0.0;
		}
	}
	
	/**
	 * return the coordinate according to the dimension 'nth'
	 * @param nth #dimension
	 * @return
	 */
	public getM_coordinate(nth:number):number {
		
		return this.m_coordinate[nth];
	}
	
	public getM_nDimensions() {
		return this.m_nDimensions;
	}
	
	/**
	 * set the coordinate according to the dimension
	 * @param nth dimension
	 * @param value value
	 */
	public setM_coordinate(nth: number, value: number): void {
		this.m_coordinate[nth] = value;
	}
	
	
}
