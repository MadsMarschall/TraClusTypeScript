"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || function (mod) {
    if (mod && mod.__esModule) return mod;
    var result = {};
    if (mod != null) for (var k in mod) if (k !== "default" && Object.prototype.hasOwnProperty.call(mod, k)) __createBinding(result, mod, k);
    __setModuleDefault(result, mod);
    return result;
};
Object.defineProperty(exports, "__esModule", { value: true });
const ClusterGen_1 = __importStar(require("./boliu/ClusterGen"));
function onEstimateParameter() {
    let p = new ClusterGen_1.Parameter();
    let generator = new ClusterGen_1.default();
    if (!generator.partitionTrajectory()) {
        console.log("Unable to partition a trajectory\n");
        return null;
    }
    if (!generator.estimateParameterValue(p)) {
        console.log("Unable to calculate the entropy\n");
        return null;
    }
    return p;
}
class TraClusterDoc {
    constructor() {
        this.m_nTrajectories = 0;
        this.m_nClusters = 0;
        this.m_clusterRatio = 0.0;
        this.m_trajectoryList = new Array();
        this.m_clusterList = new Array();
    }
    onOpenDocument(inputFileName) {
        throw new Error("Method not implemented.");
    }
    onClusterGenerate(clusterFileName, epsParam, minLnsParam) {
        throw new Error("Method not implemented.");
    }
    onEstimateParameter() {
        let p = new ClusterGen_1.Parameter();
        let generator = new ClusterGen_1.default(this);
        if (!generator.partitionTrajectory()) {
            console.log("Unable to partition a trajectory\n");
            return null;
        }
        if (!generator.estimateParameterValue(p)) {
            console.log("Unable to calculate the entropy\n");
            return null;
        }
        return p;
    }
}
let tcd = new TraClusterDoc();
let p = tcd.onEstimateParameter();
if (p != null) {
    console.log("Based on the algorithm, the suggested parameters are:\n" + "eps:" + p.epsParam + "  minLns:" + p.minLnsParam);
}
//tcd.onClusterGenerate(args[1], p.epsParam, p.minLnsParam);
//# sourceMappingURL=index.js.map