#!/bin/bash

tid=$(cat tid.txt)

echo "tid: $tid"

output=$(curl -v -X POST "https://kapi.kakao.com/v1/payment/order" \
-H 'Authorization: KakaoAK 6e8377707f3cdca4e12aadd03f2617ea' \
--data-urlencode 'cid=TC0ONETIME' \
--data-urlencode "tid=$tid")

echo "$output"