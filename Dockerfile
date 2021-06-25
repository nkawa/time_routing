# 2021/6/21最新versionを取得
FROM amd64/golang:1.15-alpine3.13
# アップデートとgitのインストール
RUN apk update && apk add git && apk add gnuplot
# appディレクトリの作成
RUN mkdir /go/src/app
# ワーキングディレクトリの設定
WORKDIR /go/src/app
# ホストのファイルをコンテナの作業ディレクトリに移行
ADD . /go/src/app
