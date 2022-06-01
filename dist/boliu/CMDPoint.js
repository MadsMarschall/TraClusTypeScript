"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
class CMDPoint {
    constructor(nDimensions) {
        this.m_nDimensions = nDimensions;
        this.m_coordinate = this.m_nDimensions;
        for (let i = 0; i < this.m_nDimensions; i++) {
            this.m_coordinate[i] = 0.0;
        }
    }
    //default constructor which shall be never used, we can use the following constructor instead
    CMDPoint() {
        this.m_nDimensions = 2;
        this.m_coordinate = this.m_nDimensions;
        this.m_coordinate[0] = this.m_coordinate[1] = 0.0;
    }
    /**
     * return the coordinate according to the dimension 'nth'
     * @param nth #dimension
     * @return
     */
    getM_coordinate(nth) {
        return this.m_coordinate[nth];
    }
    getM_nDimensions() {
        return this.m_nDimensions;
    }
    /**
     * set the coordinate according to the dimension
     * @param nth dimension
     * @param value value
     */
    setM_coordinate(nth, value) {
        this.m_coordinate[nth] = value;
    }
}
exports.default = CMDPoint;
//# sourceMappingURL=CMDPoint.js.map