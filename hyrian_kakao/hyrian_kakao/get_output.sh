#!/bin/bash

tid=$1
echo "tid: $tid"

output=$(curl -s -X POST "https://kapi.kakao.com/v1/payment/ready" \
-H "Authorization: KakaoAK 6e8377707f3cdca4e12aadd03f2617ea" \
--data-urlencode "cid=TC0ONETIME" \
--data-urlencode "partner_order_id=gpt_guide" \
--data-urlencode "partner_user_id=phg" \
--data-urlencode "item_name=손수담근 담근당근" \
--data-urlencode "quantity=1" \
--data-urlencode "total_amount=500" \
--data-urlencode "vat_amount=0" \
--data-urlencode "tax_free_amount=0" \
--data-urlencode "approval_url=https://hyrian.github.io/hyrian_kakaopay/success.html" \
--data-urlencode "cancel_url=https://hyrian.github.io/hyrian_kakaopay/cancel.html" \
--data-urlencode "fail_url=https://hyrian.github.io/hyrian_kakaopay/fail.html")

url=$(echo "$output" | jq -r '.next_redirect_mobile_url')
tid=$(echo "$output" | jq -r '.tid')
echo "tid: $tid"

# 출력할 tid 값
echo "$tid" > tid.txt

python3 generate_qr.py "$url"