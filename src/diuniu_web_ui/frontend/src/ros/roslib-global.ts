// roslib 通过 index.html 的 <script src="/roslib.min.js"> 以 UMD 全局变量加载。
// 原因：roslib 的 npm 入口 src/RosLib.js 顶层使用 `this.ROSLIB`，
// 在 ESM（this === undefined）下会直接抛 TypeError，无法作为模块导入。
const ROSLIB: any = (window as any).ROSLIB

export default ROSLIB
