# 2021/6/21最新versionを取得
FROM golang:alpine AS build-env
COPY . /work
WORKDIR /work
RUN go build -o geo-routing

FROM alpine
WORKDIR /sxbin
COPY --from=build-env /work/geo-routing /sxbin/geo-routing
COPY --from=build-env /work/map /sxbin/map
ENTRYPOINT ["/sxbin/geo-routing"]
CMD [""]
