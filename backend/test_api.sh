#!/bin/bash
BASE="http://localhost:8000"

echo "=== Test 1: Health ==="
curl -s $BASE/health | python3 -m json.tool

echo ""
echo "=== Test 2: Algorithms ==="
curl -s $BASE/api/algorithms | python3 -m json.tool

echo ""
echo "=== Test 3: Route (auto) ==="
curl -s -X POST $BASE/api/route \
      -H "Content-Type: application/json" \
        -d '{"start":{"lat":39.96,"lon":116.3},"end":{"lat":39.93,"lon":116.32}}' \
          | python3 -m json.tool

echo ""
echo "=== Test 4: Route (dijkstra) ==="
curl -s -X POST $BASE/api/route \
      -H "Content-Type: application/json" \
        -d '{"start":{"lat":39.96,"lon":116.3},"end":{"lat":39.93,"lon":116.32},"algorithm":"dijkstra"}' \
          | python3 -m json.tool

echo ""
echo "=== Test 5: Route (dijkstra_fast) ==="
curl -s -X POST $BASE/api/route \
      -H "Content-Type: application/json" \
        -d '{"start":{"lat":39.96,"lon":116.3},"end":{"lat":39.93,"lon":116.32},"algorithm":"dijkstra_fast"}' \
          | python3 -m json.tool

echo ""
echo "=== Test 6: Route (astar) ==="
curl -s -X POST $BASE/api/route \
      -H "Content-Type: application/json" \
        -d '{"start":{"lat":39.96,"lon":116.3},"end":{"lat":39.93,"lon":116.32},"algorithm":"astar"}' \
          | python3 -m json.tool

echo ""
echo "=== Test 7: Route Compare ==="
curl -s -X POST $BASE/api/route/compare \
      -H "Content-Type: application/json" \
        -d '{"start":{"lat":39.96,"lon":116.3},"end":{"lat":39.93,"lon":116.32}}' \
          | python3 -m json.tool

echo ""
echo "=== Test 8: Match ==="
curl -s -X POST $BASE/api/match \
      -H "Content-Type: application/json" \
        -d '{"lat":39.96,"lon":116.3,"k":3}' \
          | python3 -m json.tool

echo ""
echo "=== Test 9: Invalid algorithm ==="
curl -s -X POST $BASE/api/route \
      -H "Content-Type: application/json" \
        -d '{"start":{"lat":39.96,"lon":116.3},"end":{"lat":39.93,"lon":116.32},"algorithm":"invalid"}' \
          | python3 -m json.tool

echo ""
echo "=== Test 10: Invalid coordinates ==="
curl -s -X POST $BASE/api/route \
      -H "Content-Type: application/json" \
        -d '{"start":{"lat":999,"lon":116.3},"end":{"lat":39.93,"lon":116.32}}' \
          | python3 -m json.tool

echo ""
echo "=== Test 11: Out of range coordinates ==="
curl -s -X POST $BASE/api/route \
      -H "Content-Type: application/json" \
        -d '{"start":{"lat":0,"lon":0},"end":{"lat":1,"lon":1},"algorithm":"dijkstra"}' \
          | python3 -m json.tool

echo ""
echo "=== Test 12: Root path ==="
curl -s $BASE/ | python3 -m json.tool

echo ""
echo "All tests completed!"
