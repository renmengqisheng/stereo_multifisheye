# stereo_multifisheye


项目说明:

本项目参考论文https://arxiv.org/pdf/1902.11154.pdf?的思路，将单个鱼眼拆分成双针孔混合成像模型（论文中的模型为左右针孔中间柱面，由于实现略有差异，所以结果与论文不同），进而实现对多目鱼眼间的双目校正和匹配，以及生成全景鸟瞰图。



运行步骤与命令:

mkdir build 
cd build 
cmake ..
make 
./main 


