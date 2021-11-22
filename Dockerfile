# 2021/6/21最新versionを取得
FROM golang:alpine AS build-env
COPY . /work
WORKDIR /work
RUN go build

FROM alpine
WORKDIR /sxbin
# RUN apk add -u  gnuplot-x11
COPY --from=build-env /work/time_routing /sxbin/time_routing
COPY --from=build-env /work/map /sxbin/map
ENTRYPOINT ["/sxbin/time_routing"]
CMD [""]
