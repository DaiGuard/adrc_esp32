# adrc_esp32

---


## トラブルシューティング

### - PS4とペアリングしない時の対処法

PS4を別のEPS32とペアリングして、もう一度もとのESP32とペアリングするとエラーになる
これは過去にペアリングした情報がESP32に残ってしまうからだそうです

`esptool.py`を使用して一度、EPS32を初期化する必要があります

```
pip install esptool
esptool.py --chip esp32 erase_flash
```