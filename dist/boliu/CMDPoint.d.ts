export default class CMDPoint {
    private m_nDimensions;
    m_coordinate: any;
    CMDPoint(): void;
    constructor(nDimensions: any);
    /**
     * return the coordinate according to the dimension 'nth'
     * @param nth #dimension
     * @return
     */
    getM_coordinate(nth: number): number;
    getM_nDimensions(): any;
    /**
     * set the coordinate according to the dimension
     * @param nth dimension
     * @param value value
     */
    setM_coordinate(nth: number, value: number): void;
}
