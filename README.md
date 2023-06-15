# tgrobot-ros-jetson_Notebook

### 1.Ubuntu下使用命令行连接WiFi
**扫描WIFI**
``` 
sudo iwlist scan
```
**列出所有WIFI**
```
nmcli device wifi list
```
**连接指定WIFI**
```
sudo nmcli device wifi connect WiFi_NAME password WiFi_PASSWORD
```
