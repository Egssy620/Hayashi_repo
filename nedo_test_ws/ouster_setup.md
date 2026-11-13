# Ouster OS-1 静的IP設定 手順書

## 対象機器・環境

| 項目 | 値 |
|------|-----|
| LiDAR | Ouster OS-1-32-BH13 (S/N: 122538002549) |
| ファームウェア | ousteros-image-prod-bootes-v3.1.0+20240426041747 |
| ホスト | Ubuntu 22.04 (Linux 6.8.0-124-generic) |
| 接続NIC | `enp86s0` (有線ギガビット) |
| センサSDK | ouster-sdk 0.16.2 (pip install) |

## 最終構成

| 端点 | アドレス |
|------|---------|
| ホスト PC (`enp86s0`) | `192.168.1.100/24` (およびリカバリ用 `169.254.1.1/16`) |
| LiDAR (静的IP) | `192.168.1.50/24` |
| LiDAR (リンクローカル) | `169.254.21.73/16` (DHCP失敗時フォールバック) |
| LiDAR ホスト名 | `os-122538002549.local.` |
| UDPポート (LiDAR / IMU) | 7502 / 7503 |

> [!NOTE]
> WiFi (`wlo1`) は `192.168.11.x/24` セグメントでインターネット接続用に稼働中。LiDAR用セグメント (`192.168.1.x`) とは完全分離されており、両立します。

---

## 完了までのステップ

### Step 0: 事前確認

```bash
ouster-cli discover
ip addr
```

センサが検出されること、LiDARをつないだ有線NIC名を確認します。本環境では `enp86s0` でした。

### Step 1: ホストPC側 `enp86s0` に静的IPを一時割当

```bash
sudo ip addr add 192.168.1.100/24 dev enp86s0
```

確認:

```bash
ip addr show enp86s0
```

### Step 2: センサにHTTP PUT APIで静的IPを設定

センサはデフォルトでDHCP → 失敗時リンクローカルフォールバック (`169.254.x.x`) で動いています。まずこのアドレス (`169.254.21.73`) に対してPUTします。

```bash
curl -i -X PUT http://169.254.21.73/api/v1/system/network/ipv4/override \
  -H 'Content-Type: application/json' \
  --data-raw '"192.168.1.50/24"'
```

成功応答:

```
HTTP/1.1 200 OK
"192.168.1.50/24"
```

### Step 3: センサ再起動で設定反映

```bash
curl -i -X POST http://169.254.21.73/api/v1/system/restart
# → HTTP/1.1 204 No Content
```

30〜60秒待ちます。

### Step 4: 新しい静的IPで疎通確認

```bash
ping -c 3 192.168.1.50
ouster-cli discover
ouster-cli source 192.168.1.50 metadata | head -c 200
ouster-cli source 192.168.1.50 network
```

`ouster-cli source 192.168.1.50 viz` で可視化ウィンドウが開けばデータストリームも正常です(GUI環境が必要)。

### Step 5: NetworkManagerで永続化

本ホストはNetworkManager管理のため、netplanではなく `nmcli` で永続化します。

```bash
sudo nmcli connection modify enp86s0 \
  connection.autoconnect yes \
  ipv4.never-default yes
```

- `connection.autoconnect yes` ... 起動時に自動的にこのプロファイルを有効化
- `ipv4.never-default yes` ... WiFi側のデフォルトルートをLiDAR側インターフェースで上書きしない

確認:

```bash
nmcli -f ipv4.method,ipv4.addresses,ipv4.never-default,connection.autoconnect connection show enp86s0
```

期待値:

```
ipv4.method:              manual
ipv4.addresses:           169.254.1.1/16, 192.168.1.100/24
ipv4.never-default:       はい
connection.autoconnect:   はい
```

---

## トラブルシューティング・コールアウト

> [!WARNING]
> ### トラブル#1: WiFi側に `169.254.0.0/16` のルートが吸われた
>
> Step 1で `192.168.1.100/24` を割り当てた直後、`ouster-cli discover` がセンサを見つけなくなり、`ping 169.254.21.73` も届かなくなりました。
>
> **原因**: `ip route` を見ると `169.254.0.0/16 dev wlo1 scope link metric 1000` が存在し、LiDARへのリンクローカルパケットがWiFi側に送出されていました。
>
> **解決**: Step 1の後、PC側 `enp86s0` にもう一つリンクローカルアドレスを追加することで、`enp86s0` 側に `169.254.0.0/16` のスコープリンクルートが生まれ、意図した経路でLiDARに到達できるようになりました。
>
> ```bash
> sudo ip addr add 169.254.1.1/16 dev enp86s0
> ```

> [!WARNING]
> ### トラブル#2: Linuxカーネルは明示的IP割当時にIPv4LL自動設定を止める
>
> 本設定前は `enp86s0` にIPが未設定で、カーネルがIPv4リンクローカルを**自動割当**してくれていたため、センサの `169.254.21.73` に到達できていました。しかし `192.168.1.100/24` を明示的に割り当てた瞬間に、その自動link-localが消滅し、通信不能に陥りました。
>
> **解決**: トラブル#1と同じく `169.254.1.1/16` を手動追加。これをNetworkManagerプロファイルにも保持させることで、センサを工場リセットした際のリカバリ経路も確保しています。

> [!WARNING]
> ### トラブル#3: pipに `--break-system-packages` オプションがない
>
> Ubuntu 22.04添付の pip 22.0.2 では `--break-system-packages` が未サポート (pip 23.0+ で追加)。そのまま `sudo pip3 install ouster-sdk` で成功します。
>
> **教訓**: 本来はシステムPythonを触らず、venv を使うのが公式推奨。今回はシステムに直接インストールしたため、`pip` 実行時の `WARNING: Running pip as the 'root' user...` が出ますが動作上の問題はありません。

> [!WARNING]
> ### トラブル#4: `ouster-cli` 実行時の SciPy/NumPy 警告
>
> 初回 `ouster-cli discover` 時、`UserWarning: A NumPy version >=1.17.3 and <1.25.0 is required for this version of SciPy (detected version 1.26.4)` が出ました。システムの SciPy 1.8.0 と ouster-sdk 依存で入った NumPy 1.26.4 の版数不整合です。
>
> **対応**: `sudo pip3 install --upgrade scipy` で SciPy を 1.15.3 に更新 (NumPy も 2.2.6 に跳ね上がる)。
>
> **副作用**: NumPy 1.x → 2.x は破壊的変更を含むため、システムパッケージの一部が壊れる可能性があります。後で別ツールが `ImportError` を出した場合は `sudo pip3 install "numpy<2"` で戻せます。

> [!CAUTION]
> ### 注意: `sudo` を伴うコマンドは Claude から直接実行できない
>
> Claude Code の Bash ツールはTTYを持たないため、`sudo` のパスワードプロンプトに応答できません。`sudo ip addr add ...` を実行する際は、プロンプトで
>
> ```
> !sudo ip addr add 192.168.1.100/24 dev enp86s0
> ```
>
> のように `!` プレフィックスを付けてユーザ自身が実行する必要がありました。本手順中の `sudo` 行はすべてこの制約を踏まえています。

---

## 設定を元に戻す方法

### センサ側の静的IPを削除 (DHCP/リンクローカルに戻す)

```bash
curl -i -X DELETE http://192.168.1.50/api/v1/system/network/ipv4/override
curl -i -X POST http://192.168.1.50/api/v1/system/restart
```

または、ブラウザで `http://192.168.1.50` を開き、右上の **Reset Configuration** ボタンでも削除できます。

### ホスト側のIP設定を削除

```bash
sudo nmcli connection modify enp86s0 ipv4.method auto ipv4.addresses ""
sudo nmcli connection down enp86s0 && sudo nmcli connection up enp86s0
```

### 静的IPを忘れてセンサにアクセスできなくなった場合

公式手順 (WiresharkでARPを見る):

1. Wireshark を起動し、`enp86s0` を選択
2. フィルタに `arp` を入力
3. センサがARP要求しているIP (例: `169.254.128.114`) を特定
4. PC側を同じ `/24` または `/16` サブネットにしてアクセス

---

## 参考: 公式ドキュメント

- [How to set a static IP via cURL — Ouster Community Forum](https://community.ouster.com/t/how-to-set-a-static-ip-via-curl/44)
- [Ouster Firmware User Manual v3.1.0 (PDF)](https://data.ouster.io/downloads/software-user-manual/firmware-user-manual-v3.1.0.pdf)
- [Ouster Software User Manual v2.0 (PDF)](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf)
- [OUSTER OS1 USER MANUAL — ManualsLib (Section 26.2 Physical Network Setup)](https://www.manualslib.com/manual/3071555/Ouster-Os1.html)
