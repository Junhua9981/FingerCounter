# FingerCounter
### 用途: 
用mediaPipe來識別單手的數字  
手心朝鏡頭是正數 手背則是負數 (負號顯示為B) 
支援 1 ~ 9  -1 ~ -9  
帶有顯示fps(通常>29)  
識別有透過矩陣轉換成正的(即手腕在下中指朝上)  
  
### Usage:  
1. `python BetterFingerCounter.py`  
2. ESC鍵退出  
  
### TODO:  
- [ ] 用mediaPipe內建的方式識別左右手  
- [ ] 低像素下識別不佳，有顯著抖動  
  
### Dependecy:  
mediapipe==0.8.7.2  
更新版本的Detect API好像有改，會出bug  
  
### 備註:  
很隨意的一個小小東西   
code也很醜  
以後有時間再重構QQ  
