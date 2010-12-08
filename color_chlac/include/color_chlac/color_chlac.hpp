// inline int pos( float val ){
//   if( val < -0.5 ) return 0;
//   if( val <  0.5 ) return 1;
//   return 2;
// }

inline int reverse( int val ){
  return 255 - val;
}

inline int
pcl::ColorCHLACEstimation::binarize_r ( int val )
{
  if( val > color_thR ) return 1;
  return 0;
}
inline int
pcl::ColorCHLACEstimation::binarize_g ( int val )
{
  if( val > color_thG ) return 1;
  return 0;
}
inline int
pcl::ColorCHLACEstimation::binarize_b ( int val )
{
  if( val > color_thB ) return 1;
  return 0;
}

inline void 
pcl::ColorCHLACEstimation::addColorCHLAC_0 ( PointCloudOut &output )
{
  output.points[0].histogram[   0 ] += center_r;
  output.points[0].histogram[   1 ] += center_r_;
  output.points[0].histogram[   2 ] += center_g;
  output.points[0].histogram[   3 ] += center_g_;
  output.points[0].histogram[   4 ] += center_b;
  output.points[0].histogram[   5 ] += center_b_;
  output.points[0].histogram[ 474 ] +=     center_r * center_r;
  output.points[0].histogram[ 475 ] += 2 * center_r * center_r_;
  output.points[0].histogram[ 476 ] += 2 * center_r * center_g;
  output.points[0].histogram[ 477 ] += 2 * center_r * center_g_;
  output.points[0].histogram[ 478 ] += 2 * center_r * center_b;
  output.points[0].histogram[ 479 ] += 2 * center_r * center_b_;
  output.points[0].histogram[ 480 ] +=     center_r_* center_r_;
  output.points[0].histogram[ 481 ] += 2 * center_r_* center_g;
  output.points[0].histogram[ 482 ] += 2 * center_r_* center_g_;
  output.points[0].histogram[ 483 ] += 2 * center_r_* center_b;
  output.points[0].histogram[ 484 ] += 2 * center_r_* center_b_;
  output.points[0].histogram[ 485 ] +=     center_g * center_g;
  output.points[0].histogram[ 486 ] += 2 * center_g * center_g_;
  output.points[0].histogram[ 487 ] += 2 * center_g * center_b;
  output.points[0].histogram[ 488 ] += 2 * center_g * center_b_;
  output.points[0].histogram[ 489 ] +=     center_g_* center_g_;
  output.points[0].histogram[ 490 ] += 2 * center_g_* center_b;
  output.points[0].histogram[ 491 ] += 2 * center_g_* center_b_;
  output.points[0].histogram[ 492 ] +=     center_b * center_b;
  output.points[0].histogram[ 493 ] += 2 * center_b * center_b_;
  output.points[0].histogram[ 494 ] +=     center_b_* center_b_;
}

inline void 
pcl::ColorCHLACEstimation::addColorCHLAC_0_bin ( PointCloudOut &output )
{
  if( center_bin_r )
    output.points[0].histogram[ 495 ] ++;
  else
    output.points[0].histogram[ 496 ] ++;
  if( center_bin_g )
    output.points[0].histogram[ 497 ] ++;
  else
    output.points[0].histogram[ 498 ] ++;
  if( center_bin_b )
    output.points[0].histogram[ 499 ] ++;
  else
    output.points[0].histogram[ 500 ] ++;

  if( center_bin_r ){
    if( center_bin_g )
      output.points[0].histogram[ 969 ] ++;
    else
      output.points[0].histogram[ 970 ] ++;
    if( center_bin_b )
      output.points[0].histogram[ 971 ] ++;
    else
      output.points[0].histogram[ 972 ] ++;
  }
  else{
    if( center_bin_g )
      output.points[0].histogram[ 973 ] ++;
    else
      output.points[0].histogram[ 974 ] ++;
    if( center_bin_b )
      output.points[0].histogram[ 975 ] ++;
    else
      output.points[0].histogram[ 976 ] ++;
  }
  if( center_bin_g ){
    if( center_bin_b )
      output.points[0].histogram[ 977 ] ++;
    else
      output.points[0].histogram[ 978 ] ++;
  }
  else{
    if( center_bin_b )
      output.points[0].histogram[ 979 ] ++;
    else
      output.points[0].histogram[ 980 ] ++;
  }
}

inline void 
pcl::ColorCHLACEstimation::addColorCHLAC_1 ( PointCloudOut &output, int neighbor_idx, int r, int g, int b )
{
  const int r_ = reverse( r );
  const int g_ = reverse( g );
  const int b_ = reverse( b );

  switch( neighbor_idx ){
  case 0:
    output.points[0].histogram[   6 ] += center_r * r;
    output.points[0].histogram[  15 ] += center_r * r_;
    output.points[0].histogram[  24 ] += center_r * g;
    output.points[0].histogram[  33 ] += center_r * g_;
    output.points[0].histogram[  42 ] += center_r * b;
    output.points[0].histogram[  51 ] += center_r * b_;
    output.points[0].histogram[  84 ] += center_r_* r;
    output.points[0].histogram[  93 ] += center_r_* r_;
    output.points[0].histogram[ 102 ] += center_r_* g;
    output.points[0].histogram[ 111 ] += center_r_* g_;
    output.points[0].histogram[ 120 ] += center_r_* b;
    output.points[0].histogram[ 129 ] += center_r_* b_;
    output.points[0].histogram[ 162 ] += center_g * r;
    output.points[0].histogram[ 171 ] += center_g * r_;
    output.points[0].histogram[ 180 ] += center_g * g;
    output.points[0].histogram[ 189 ] += center_g * g_;
    output.points[0].histogram[ 198 ] += center_g * b;
    output.points[0].histogram[ 207 ] += center_g * b_;
    output.points[0].histogram[ 240 ] += center_g_* r;
    output.points[0].histogram[ 249 ] += center_g_* r_;
    output.points[0].histogram[ 258 ] += center_g_* g;
    output.points[0].histogram[ 267 ] += center_g_* g_;
    output.points[0].histogram[ 276 ] += center_g_* b;
    output.points[0].histogram[ 285 ] += center_g_* b_;
    output.points[0].histogram[ 318 ] += center_b * r;
    output.points[0].histogram[ 327 ] += center_b * r_;
    output.points[0].histogram[ 336 ] += center_b * g;
    output.points[0].histogram[ 345 ] += center_b * g_;
    output.points[0].histogram[ 354 ] += center_b * b;
    output.points[0].histogram[ 363 ] += center_b * b_;
    output.points[0].histogram[ 396 ] += center_b_* r;
    output.points[0].histogram[ 405 ] += center_b_* r_;
    output.points[0].histogram[ 414 ] += center_b_* g;
    output.points[0].histogram[ 423 ] += center_b_* g_;
    output.points[0].histogram[ 432 ] += center_b_* b;
    output.points[0].histogram[ 441 ] += center_b_* b_;
    break;
  case 1:
    output.points[0].histogram[   7 ] += center_r * r;
    output.points[0].histogram[  16 ] += center_r * r_;
    output.points[0].histogram[  25 ] += center_r * g;
    output.points[0].histogram[  34 ] += center_r * g_;
    output.points[0].histogram[  43 ] += center_r * b;
    output.points[0].histogram[  52 ] += center_r * b_;
    output.points[0].histogram[  85 ] += center_r_* r;
    output.points[0].histogram[  94 ] += center_r_* r_;
    output.points[0].histogram[ 103 ] += center_r_* g;
    output.points[0].histogram[ 112 ] += center_r_* g_;
    output.points[0].histogram[ 121 ] += center_r_* b;
    output.points[0].histogram[ 130 ] += center_r_* b_;
    output.points[0].histogram[ 163 ] += center_g * r;
    output.points[0].histogram[ 172 ] += center_g * r_;
    output.points[0].histogram[ 181 ] += center_g * g;
    output.points[0].histogram[ 190 ] += center_g * g_;
    output.points[0].histogram[ 199 ] += center_g * b;
    output.points[0].histogram[ 208 ] += center_g * b_;
    output.points[0].histogram[ 241 ] += center_g_* r;
    output.points[0].histogram[ 250 ] += center_g_* r_;
    output.points[0].histogram[ 259 ] += center_g_* g;
    output.points[0].histogram[ 268 ] += center_g_* g_;
    output.points[0].histogram[ 277 ] += center_g_* b;
    output.points[0].histogram[ 286 ] += center_g_* b_;
    output.points[0].histogram[ 319 ] += center_b * r;
    output.points[0].histogram[ 328 ] += center_b * r_;
    output.points[0].histogram[ 337 ] += center_b * g;
    output.points[0].histogram[ 346 ] += center_b * g_;
    output.points[0].histogram[ 355 ] += center_b * b;
    output.points[0].histogram[ 364 ] += center_b * b_;
    output.points[0].histogram[ 397 ] += center_b_* r;
    output.points[0].histogram[ 406 ] += center_b_* r_;
    output.points[0].histogram[ 415 ] += center_b_* g;
    output.points[0].histogram[ 424 ] += center_b_* g_;
    output.points[0].histogram[ 433 ] += center_b_* b;
    output.points[0].histogram[ 442 ] += center_b_* b_;
    break;
  case 2:
    output.points[0].histogram[   8 ] += center_r * r;
    output.points[0].histogram[  17 ] += center_r * r_;
    output.points[0].histogram[  26 ] += center_r * g;
    output.points[0].histogram[  35 ] += center_r * g_;
    output.points[0].histogram[  44 ] += center_r * b;
    output.points[0].histogram[  53 ] += center_r * b_;
    output.points[0].histogram[  86 ] += center_r_* r;
    output.points[0].histogram[  95 ] += center_r_* r_;
    output.points[0].histogram[ 104 ] += center_r_* g;
    output.points[0].histogram[ 113 ] += center_r_* g_;
    output.points[0].histogram[ 122 ] += center_r_* b;
    output.points[0].histogram[ 131 ] += center_r_* b_;
    output.points[0].histogram[ 164 ] += center_g * r;
    output.points[0].histogram[ 173 ] += center_g * r_;
    output.points[0].histogram[ 182 ] += center_g * g;
    output.points[0].histogram[ 191 ] += center_g * g_;
    output.points[0].histogram[ 200 ] += center_g * b;
    output.points[0].histogram[ 209 ] += center_g * b_;
    output.points[0].histogram[ 242 ] += center_g_* r;
    output.points[0].histogram[ 251 ] += center_g_* r_;
    output.points[0].histogram[ 260 ] += center_g_* g;
    output.points[0].histogram[ 269 ] += center_g_* g_;
    output.points[0].histogram[ 278 ] += center_g_* b;
    output.points[0].histogram[ 287 ] += center_g_* b_;
    output.points[0].histogram[ 320 ] += center_b * r;
    output.points[0].histogram[ 329 ] += center_b * r_;
    output.points[0].histogram[ 338 ] += center_b * g;
    output.points[0].histogram[ 347 ] += center_b * g_;
    output.points[0].histogram[ 356 ] += center_b * b;
    output.points[0].histogram[ 365 ] += center_b * b_;
    output.points[0].histogram[ 398 ] += center_b_* r;
    output.points[0].histogram[ 407 ] += center_b_* r_;
    output.points[0].histogram[ 416 ] += center_b_* g;
    output.points[0].histogram[ 425 ] += center_b_* g_;
    output.points[0].histogram[ 434 ] += center_b_* b;
    output.points[0].histogram[ 443 ] += center_b_* b_;
    break;
  case 3:
    output.points[0].histogram[   9 ] += center_r * r;
    output.points[0].histogram[  18 ] += center_r * r_;
    output.points[0].histogram[  27 ] += center_r * g;
    output.points[0].histogram[  36 ] += center_r * g_;
    output.points[0].histogram[  45 ] += center_r * b;
    output.points[0].histogram[  54 ] += center_r * b_;
    output.points[0].histogram[  87 ] += center_r_* r;
    output.points[0].histogram[  96 ] += center_r_* r_;
    output.points[0].histogram[ 105 ] += center_r_* g;
    output.points[0].histogram[ 114 ] += center_r_* g_;
    output.points[0].histogram[ 123 ] += center_r_* b;
    output.points[0].histogram[ 132 ] += center_r_* b_;
    output.points[0].histogram[ 165 ] += center_g * r;
    output.points[0].histogram[ 174 ] += center_g * r_;
    output.points[0].histogram[ 183 ] += center_g * g;
    output.points[0].histogram[ 192 ] += center_g * g_;
    output.points[0].histogram[ 201 ] += center_g * b;
    output.points[0].histogram[ 210 ] += center_g * b_;
    output.points[0].histogram[ 243 ] += center_g_* r;
    output.points[0].histogram[ 252 ] += center_g_* r_;
    output.points[0].histogram[ 261 ] += center_g_* g;
    output.points[0].histogram[ 270 ] += center_g_* g_;
    output.points[0].histogram[ 279 ] += center_g_* b;
    output.points[0].histogram[ 288 ] += center_g_* b_;
    output.points[0].histogram[ 321 ] += center_b * r;
    output.points[0].histogram[ 330 ] += center_b * r_;
    output.points[0].histogram[ 339 ] += center_b * g;
    output.points[0].histogram[ 348 ] += center_b * g_;
    output.points[0].histogram[ 357 ] += center_b * b;
    output.points[0].histogram[ 366 ] += center_b * b_;
    output.points[0].histogram[ 399 ] += center_b_* r;
    output.points[0].histogram[ 408 ] += center_b_* r_;
    output.points[0].histogram[ 417 ] += center_b_* g;
    output.points[0].histogram[ 426 ] += center_b_* g_;
    output.points[0].histogram[ 435 ] += center_b_* b;
    output.points[0].histogram[ 444 ] += center_b_* b_;
    break;
  case 4:
    output.points[0].histogram[  10 ] += center_r * r;
    output.points[0].histogram[  19 ] += center_r * r_;
    output.points[0].histogram[  28 ] += center_r * g;
    output.points[0].histogram[  37 ] += center_r * g_;
    output.points[0].histogram[  46 ] += center_r * b;
    output.points[0].histogram[  55 ] += center_r * b_;
    output.points[0].histogram[  88 ] += center_r_* r;
    output.points[0].histogram[  97 ] += center_r_* r_;
    output.points[0].histogram[ 106 ] += center_r_* g;
    output.points[0].histogram[ 115 ] += center_r_* g_;
    output.points[0].histogram[ 124 ] += center_r_* b;
    output.points[0].histogram[ 133 ] += center_r_* b_;
    output.points[0].histogram[ 166 ] += center_g * r;
    output.points[0].histogram[ 175 ] += center_g * r_;
    output.points[0].histogram[ 184 ] += center_g * g;
    output.points[0].histogram[ 193 ] += center_g * g_;
    output.points[0].histogram[ 202 ] += center_g * b;
    output.points[0].histogram[ 211 ] += center_g * b_;
    output.points[0].histogram[ 244 ] += center_g_* r;
    output.points[0].histogram[ 253 ] += center_g_* r_;
    output.points[0].histogram[ 262 ] += center_g_* g;
    output.points[0].histogram[ 271 ] += center_g_* g_;
    output.points[0].histogram[ 280 ] += center_g_* b;
    output.points[0].histogram[ 289 ] += center_g_* b_;
    output.points[0].histogram[ 322 ] += center_b * r;
    output.points[0].histogram[ 331 ] += center_b * r_;
    output.points[0].histogram[ 340 ] += center_b * g;
    output.points[0].histogram[ 349 ] += center_b * g_;
    output.points[0].histogram[ 358 ] += center_b * b;
    output.points[0].histogram[ 367 ] += center_b * b_;
    output.points[0].histogram[ 400 ] += center_b_* r;
    output.points[0].histogram[ 409 ] += center_b_* r_;
    output.points[0].histogram[ 418 ] += center_b_* g;
    output.points[0].histogram[ 427 ] += center_b_* g_;
    output.points[0].histogram[ 436 ] += center_b_* b;
    output.points[0].histogram[ 445 ] += center_b_* b_;
    break;
  case 5:
    output.points[0].histogram[  11 ] += center_r * r;
    output.points[0].histogram[  20 ] += center_r * r_;
    output.points[0].histogram[  29 ] += center_r * g;
    output.points[0].histogram[  38 ] += center_r * g_;
    output.points[0].histogram[  47 ] += center_r * b;
    output.points[0].histogram[  56 ] += center_r * b_;
    output.points[0].histogram[  89 ] += center_r_* r;
    output.points[0].histogram[  98 ] += center_r_* r_;
    output.points[0].histogram[ 107 ] += center_r_* g;
    output.points[0].histogram[ 116 ] += center_r_* g_;
    output.points[0].histogram[ 125 ] += center_r_* b;
    output.points[0].histogram[ 134 ] += center_r_* b_;
    output.points[0].histogram[ 167 ] += center_g * r;
    output.points[0].histogram[ 176 ] += center_g * r_;
    output.points[0].histogram[ 185 ] += center_g * g;
    output.points[0].histogram[ 194 ] += center_g * g_;
    output.points[0].histogram[ 203 ] += center_g * b;
    output.points[0].histogram[ 212 ] += center_g * b_;
    output.points[0].histogram[ 245 ] += center_g_* r;
    output.points[0].histogram[ 254 ] += center_g_* r_;
    output.points[0].histogram[ 263 ] += center_g_* g;
    output.points[0].histogram[ 272 ] += center_g_* g_;
    output.points[0].histogram[ 281 ] += center_g_* b;
    output.points[0].histogram[ 290 ] += center_g_* b_;
    output.points[0].histogram[ 323 ] += center_b * r;
    output.points[0].histogram[ 332 ] += center_b * r_;
    output.points[0].histogram[ 341 ] += center_b * g;
    output.points[0].histogram[ 350 ] += center_b * g_;
    output.points[0].histogram[ 359 ] += center_b * b;
    output.points[0].histogram[ 368 ] += center_b * b_;
    output.points[0].histogram[ 401 ] += center_b_* r;
    output.points[0].histogram[ 410 ] += center_b_* r_;
    output.points[0].histogram[ 419 ] += center_b_* g;
    output.points[0].histogram[ 428 ] += center_b_* g_;
    output.points[0].histogram[ 437 ] += center_b_* b;
    output.points[0].histogram[ 446 ] += center_b_* b_;
    break;
  case 6:
    output.points[0].histogram[  12 ] += center_r * r;
    output.points[0].histogram[  21 ] += center_r * r_;
    output.points[0].histogram[  30 ] += center_r * g;
    output.points[0].histogram[  39 ] += center_r * g_;
    output.points[0].histogram[  48 ] += center_r * b;
    output.points[0].histogram[  57 ] += center_r * b_;
    output.points[0].histogram[  90 ] += center_r_* r;
    output.points[0].histogram[  99 ] += center_r_* r_;
    output.points[0].histogram[ 108 ] += center_r_* g;
    output.points[0].histogram[ 117 ] += center_r_* g_;
    output.points[0].histogram[ 126 ] += center_r_* b;
    output.points[0].histogram[ 135 ] += center_r_* b_;
    output.points[0].histogram[ 168 ] += center_g * r;
    output.points[0].histogram[ 177 ] += center_g * r_;
    output.points[0].histogram[ 186 ] += center_g * g;
    output.points[0].histogram[ 195 ] += center_g * g_;
    output.points[0].histogram[ 204 ] += center_g * b;
    output.points[0].histogram[ 213 ] += center_g * b_;
    output.points[0].histogram[ 246 ] += center_g_* r;
    output.points[0].histogram[ 255 ] += center_g_* r_;
    output.points[0].histogram[ 264 ] += center_g_* g;
    output.points[0].histogram[ 273 ] += center_g_* g_;
    output.points[0].histogram[ 282 ] += center_g_* b;
    output.points[0].histogram[ 291 ] += center_g_* b_;
    output.points[0].histogram[ 324 ] += center_b * r;
    output.points[0].histogram[ 333 ] += center_b * r_;
    output.points[0].histogram[ 342 ] += center_b * g;
    output.points[0].histogram[ 351 ] += center_b * g_;
    output.points[0].histogram[ 360 ] += center_b * b;
    output.points[0].histogram[ 369 ] += center_b * b_;
    output.points[0].histogram[ 402 ] += center_b_* r;
    output.points[0].histogram[ 411 ] += center_b_* r_;
    output.points[0].histogram[ 420 ] += center_b_* g;
    output.points[0].histogram[ 429 ] += center_b_* g_;
    output.points[0].histogram[ 438 ] += center_b_* b;
    output.points[0].histogram[ 447 ] += center_b_* b_;
    break;
  case 7:
    output.points[0].histogram[  13 ] += center_r * r;
    output.points[0].histogram[  22 ] += center_r * r_;
    output.points[0].histogram[  31 ] += center_r * g;
    output.points[0].histogram[  40 ] += center_r * g_;
    output.points[0].histogram[  49 ] += center_r * b;
    output.points[0].histogram[  58 ] += center_r * b_;
    output.points[0].histogram[  91 ] += center_r_* r;
    output.points[0].histogram[ 100 ] += center_r_* r_;
    output.points[0].histogram[ 109 ] += center_r_* g;
    output.points[0].histogram[ 118 ] += center_r_* g_;
    output.points[0].histogram[ 127 ] += center_r_* b;
    output.points[0].histogram[ 136 ] += center_r_* b_;
    output.points[0].histogram[ 169 ] += center_g * r;
    output.points[0].histogram[ 178 ] += center_g * r_;
    output.points[0].histogram[ 187 ] += center_g * g;
    output.points[0].histogram[ 196 ] += center_g * g_;
    output.points[0].histogram[ 205 ] += center_g * b;
    output.points[0].histogram[ 214 ] += center_g * b_;
    output.points[0].histogram[ 247 ] += center_g_* r;
    output.points[0].histogram[ 256 ] += center_g_* r_;
    output.points[0].histogram[ 265 ] += center_g_* g;
    output.points[0].histogram[ 274 ] += center_g_* g_;
    output.points[0].histogram[ 283 ] += center_g_* b;
    output.points[0].histogram[ 292 ] += center_g_* b_;
    output.points[0].histogram[ 325 ] += center_b * r;
    output.points[0].histogram[ 334 ] += center_b * r_;
    output.points[0].histogram[ 343 ] += center_b * g;
    output.points[0].histogram[ 352 ] += center_b * g_;
    output.points[0].histogram[ 361 ] += center_b * b;
    output.points[0].histogram[ 370 ] += center_b * b_;
    output.points[0].histogram[ 403 ] += center_b_* r;
    output.points[0].histogram[ 412 ] += center_b_* r_;
    output.points[0].histogram[ 421 ] += center_b_* g;
    output.points[0].histogram[ 430 ] += center_b_* g_;
    output.points[0].histogram[ 439 ] += center_b_* b;
    output.points[0].histogram[ 448 ] += center_b_* b_;
    break;
  case 8:
    output.points[0].histogram[  14 ] += center_r * r;
    output.points[0].histogram[  23 ] += center_r * r_;
    output.points[0].histogram[  32 ] += center_r * g;
    output.points[0].histogram[  41 ] += center_r * g_;
    output.points[0].histogram[  50 ] += center_r * b;
    output.points[0].histogram[  59 ] += center_r * b_;
    output.points[0].histogram[  92 ] += center_r_* r;
    output.points[0].histogram[ 101 ] += center_r_* r_;
    output.points[0].histogram[ 110 ] += center_r_* g;
    output.points[0].histogram[ 119 ] += center_r_* g_;
    output.points[0].histogram[ 128 ] += center_r_* b;
    output.points[0].histogram[ 137 ] += center_r_* b_;
    output.points[0].histogram[ 170 ] += center_g * r;
    output.points[0].histogram[ 179 ] += center_g * r_;
    output.points[0].histogram[ 188 ] += center_g * g;
    output.points[0].histogram[ 197 ] += center_g * g_;
    output.points[0].histogram[ 206 ] += center_g * b;
    output.points[0].histogram[ 215 ] += center_g * b_;
    output.points[0].histogram[ 248 ] += center_g_* r;
    output.points[0].histogram[ 257 ] += center_g_* r_;
    output.points[0].histogram[ 266 ] += center_g_* g;
    output.points[0].histogram[ 275 ] += center_g_* g_;
    output.points[0].histogram[ 284 ] += center_g_* b;
    output.points[0].histogram[ 293 ] += center_g_* b_;
    output.points[0].histogram[ 326 ] += center_b * r;
    output.points[0].histogram[ 335 ] += center_b * r_;
    output.points[0].histogram[ 344 ] += center_b * g;
    output.points[0].histogram[ 353 ] += center_b * g_;
    output.points[0].histogram[ 362 ] += center_b * b;
    output.points[0].histogram[ 371 ] += center_b * b_;
    output.points[0].histogram[ 404 ] += center_b_* r;
    output.points[0].histogram[ 413 ] += center_b_* r_;
    output.points[0].histogram[ 422 ] += center_b_* g;
    output.points[0].histogram[ 431 ] += center_b_* g_;
    output.points[0].histogram[ 440 ] += center_b_* b;
    output.points[0].histogram[ 449 ] += center_b_* b_;
    break;
  case 9:
    output.points[0].histogram[  60 ] += center_r * r;
    output.points[0].histogram[  64 ] += center_r * r_;
    output.points[0].histogram[  68 ] += center_r * g;
    output.points[0].histogram[  72 ] += center_r * g_;
    output.points[0].histogram[  76 ] += center_r * b;
    output.points[0].histogram[  80 ] += center_r * b_;
    output.points[0].histogram[ 138 ] += center_r_* r;
    output.points[0].histogram[ 142 ] += center_r_* r_;
    output.points[0].histogram[ 146 ] += center_r_* g;
    output.points[0].histogram[ 150 ] += center_r_* g_;
    output.points[0].histogram[ 154 ] += center_r_* b;
    output.points[0].histogram[ 158 ] += center_r_* b_;
    output.points[0].histogram[ 216 ] += center_g * r;
    output.points[0].histogram[ 220 ] += center_g * r_;
    output.points[0].histogram[ 224 ] += center_g * g;
    output.points[0].histogram[ 228 ] += center_g * g_;
    output.points[0].histogram[ 232 ] += center_g * b;
    output.points[0].histogram[ 236 ] += center_g * b_;
    output.points[0].histogram[ 294 ] += center_g_* r;
    output.points[0].histogram[ 298 ] += center_g_* r_;
    output.points[0].histogram[ 302 ] += center_g_* g;
    output.points[0].histogram[ 306 ] += center_g_* g_;
    output.points[0].histogram[ 310 ] += center_g_* b;
    output.points[0].histogram[ 314 ] += center_g_* b_;
    output.points[0].histogram[ 372 ] += center_b * r;
    output.points[0].histogram[ 376 ] += center_b * r_;
    output.points[0].histogram[ 380 ] += center_b * g;
    output.points[0].histogram[ 384 ] += center_b * g_;
    output.points[0].histogram[ 388 ] += center_b * b;
    output.points[0].histogram[ 392 ] += center_b * b_;
    output.points[0].histogram[ 450 ] += center_b_* r;
    output.points[0].histogram[ 454 ] += center_b_* r_;
    output.points[0].histogram[ 458 ] += center_b_* g;
    output.points[0].histogram[ 462 ] += center_b_* g_;
    output.points[0].histogram[ 466 ] += center_b_* b;
    output.points[0].histogram[ 470 ] += center_b_* b_;
    break;
  case 10:
    output.points[0].histogram[  61 ] += center_r * r;
    output.points[0].histogram[  65 ] += center_r * r_;
    output.points[0].histogram[  69 ] += center_r * g;
    output.points[0].histogram[  73 ] += center_r * g_;
    output.points[0].histogram[  77 ] += center_r * b;
    output.points[0].histogram[  81 ] += center_r * b_;
    output.points[0].histogram[ 139 ] += center_r_* r;
    output.points[0].histogram[ 143 ] += center_r_* r_;
    output.points[0].histogram[ 147 ] += center_r_* g;
    output.points[0].histogram[ 151 ] += center_r_* g_;
    output.points[0].histogram[ 155 ] += center_r_* b;
    output.points[0].histogram[ 159 ] += center_r_* b_;
    output.points[0].histogram[ 217 ] += center_g * r;
    output.points[0].histogram[ 221 ] += center_g * r_;
    output.points[0].histogram[ 225 ] += center_g * g;
    output.points[0].histogram[ 229 ] += center_g * g_;
    output.points[0].histogram[ 233 ] += center_g * b;
    output.points[0].histogram[ 237 ] += center_g * b_;
    output.points[0].histogram[ 295 ] += center_g_* r;
    output.points[0].histogram[ 299 ] += center_g_* r_;
    output.points[0].histogram[ 303 ] += center_g_* g;
    output.points[0].histogram[ 307 ] += center_g_* g_;
    output.points[0].histogram[ 311 ] += center_g_* b;
    output.points[0].histogram[ 315 ] += center_g_* b_;
    output.points[0].histogram[ 373 ] += center_b * r;
    output.points[0].histogram[ 377 ] += center_b * r_;
    output.points[0].histogram[ 381 ] += center_b * g;
    output.points[0].histogram[ 385 ] += center_b * g_;
    output.points[0].histogram[ 389 ] += center_b * b;
    output.points[0].histogram[ 393 ] += center_b * b_;
    output.points[0].histogram[ 451 ] += center_b_* r;
    output.points[0].histogram[ 455 ] += center_b_* r_;
    output.points[0].histogram[ 459 ] += center_b_* g;
    output.points[0].histogram[ 463 ] += center_b_* g_;
    output.points[0].histogram[ 467 ] += center_b_* b;
    output.points[0].histogram[ 471 ] += center_b_* b_;
    break;
  case 11:
    output.points[0].histogram[  62 ] += center_r * r;
    output.points[0].histogram[  66 ] += center_r * r_;
    output.points[0].histogram[  70 ] += center_r * g;
    output.points[0].histogram[  74 ] += center_r * g_;
    output.points[0].histogram[  78 ] += center_r * b;
    output.points[0].histogram[  82 ] += center_r * b_;
    output.points[0].histogram[ 140 ] += center_r_* r;
    output.points[0].histogram[ 144 ] += center_r_* r_;
    output.points[0].histogram[ 148 ] += center_r_* g;
    output.points[0].histogram[ 152 ] += center_r_* g_;
    output.points[0].histogram[ 156 ] += center_r_* b;
    output.points[0].histogram[ 160 ] += center_r_* b_;
    output.points[0].histogram[ 218 ] += center_g * r;
    output.points[0].histogram[ 222 ] += center_g * r_;
    output.points[0].histogram[ 226 ] += center_g * g;
    output.points[0].histogram[ 230 ] += center_g * g_;
    output.points[0].histogram[ 234 ] += center_g * b;
    output.points[0].histogram[ 238 ] += center_g * b_;
    output.points[0].histogram[ 296 ] += center_g_* r;
    output.points[0].histogram[ 300 ] += center_g_* r_;
    output.points[0].histogram[ 304 ] += center_g_* g;
    output.points[0].histogram[ 308 ] += center_g_* g_;
    output.points[0].histogram[ 312 ] += center_g_* b;
    output.points[0].histogram[ 316 ] += center_g_* b_;
    output.points[0].histogram[ 374 ] += center_b * r;
    output.points[0].histogram[ 378 ] += center_b * r_;
    output.points[0].histogram[ 382 ] += center_b * g;
    output.points[0].histogram[ 386 ] += center_b * g_;
    output.points[0].histogram[ 390 ] += center_b * b;
    output.points[0].histogram[ 394 ] += center_b * b_;
    output.points[0].histogram[ 452 ] += center_b_* r;
    output.points[0].histogram[ 456 ] += center_b_* r_;
    output.points[0].histogram[ 460 ] += center_b_* g;
    output.points[0].histogram[ 464 ] += center_b_* g_;
    output.points[0].histogram[ 468 ] += center_b_* b;
    output.points[0].histogram[ 472 ] += center_b_* b_;
    break;
  case 12:
    output.points[0].histogram[  63 ] += center_r * r;
    output.points[0].histogram[  67 ] += center_r * r_;
    output.points[0].histogram[  71 ] += center_r * g;
    output.points[0].histogram[  75 ] += center_r * g_;
    output.points[0].histogram[  79 ] += center_r * b;
    output.points[0].histogram[  83 ] += center_r * b_;
    output.points[0].histogram[ 141 ] += center_r_* r;
    output.points[0].histogram[ 145 ] += center_r_* r_;
    output.points[0].histogram[ 149 ] += center_r_* g;
    output.points[0].histogram[ 153 ] += center_r_* g_;
    output.points[0].histogram[ 157 ] += center_r_* b;
    output.points[0].histogram[ 161 ] += center_r_* b_;
    output.points[0].histogram[ 219 ] += center_g * r;
    output.points[0].histogram[ 223 ] += center_g * r_;
    output.points[0].histogram[ 227 ] += center_g * g;
    output.points[0].histogram[ 231 ] += center_g * g_;
    output.points[0].histogram[ 235 ] += center_g * b;
    output.points[0].histogram[ 239 ] += center_g * b_;
    output.points[0].histogram[ 297 ] += center_g_* r;
    output.points[0].histogram[ 301 ] += center_g_* r_;
    output.points[0].histogram[ 305 ] += center_g_* g;
    output.points[0].histogram[ 309 ] += center_g_* g_;
    output.points[0].histogram[ 313 ] += center_g_* b;
    output.points[0].histogram[ 317 ] += center_g_* b_;
    output.points[0].histogram[ 375 ] += center_b * r;
    output.points[0].histogram[ 379 ] += center_b * r_;
    output.points[0].histogram[ 383 ] += center_b * g;
    output.points[0].histogram[ 387 ] += center_b * g_;
    output.points[0].histogram[ 391 ] += center_b * b;
    output.points[0].histogram[ 395 ] += center_b * b_;
    output.points[0].histogram[ 453 ] += center_b_* r;
    output.points[0].histogram[ 457 ] += center_b_* r_;
    output.points[0].histogram[ 461 ] += center_b_* g;
    output.points[0].histogram[ 465 ] += center_b_* g_;
    output.points[0].histogram[ 469 ] += center_b_* b;
    output.points[0].histogram[ 473 ] += center_b_* b_;
    break;
    //   case 13: // itself
    //     break;
    //   case 14: // 14 - 26: redundant
    //     break;
  default:
    break;
  }
}

inline void 
pcl::ColorCHLACEstimation::addColorCHLAC_1_bin ( PointCloudOut &output, int neighbor_idx, int r, int g, int b )
{
  const int shift_dim = 495;
  const int r_ = 1 - r;
  const int g_ = 1 - g;
  const int b_ = 1 - b;

  switch( neighbor_idx ){
  case 0:
    if( center_bin_r ){
      output.points[0].histogram[ shift_dim +   6 ] += r;
      output.points[0].histogram[ shift_dim +  15 ] += r_;
      output.points[0].histogram[ shift_dim +  24 ] += g;
      output.points[0].histogram[ shift_dim +  33 ] += g_;
      output.points[0].histogram[ shift_dim +  42 ] += b;
      output.points[0].histogram[ shift_dim +  51 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim +  84 ] += r;
      output.points[0].histogram[ shift_dim +  93 ] += r_;
      output.points[0].histogram[ shift_dim + 102 ] += g;
      output.points[0].histogram[ shift_dim + 111 ] += g_;
      output.points[0].histogram[ shift_dim + 120 ] += b;
      output.points[0].histogram[ shift_dim + 129 ] += b_;
    }
    if( center_bin_g ){
      output.points[0].histogram[ shift_dim + 162 ] += r;
      output.points[0].histogram[ shift_dim + 171 ] += r_;
      output.points[0].histogram[ shift_dim + 180 ] += g;
      output.points[0].histogram[ shift_dim + 189 ] += g_;
      output.points[0].histogram[ shift_dim + 198 ] += b;
      output.points[0].histogram[ shift_dim + 207 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 240 ] += r;
      output.points[0].histogram[ shift_dim + 249 ] += r_;
      output.points[0].histogram[ shift_dim + 258 ] += g;
      output.points[0].histogram[ shift_dim + 267 ] += g_;
      output.points[0].histogram[ shift_dim + 276 ] += b;
      output.points[0].histogram[ shift_dim + 285 ] += b_;
    }
    if( center_bin_b ){
      output.points[0].histogram[ shift_dim + 318 ] += r;
      output.points[0].histogram[ shift_dim + 327 ] += r_;
      output.points[0].histogram[ shift_dim + 336 ] += g;
      output.points[0].histogram[ shift_dim + 345 ] += g_;
      output.points[0].histogram[ shift_dim + 354 ] += b;
      output.points[0].histogram[ shift_dim + 363 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 396 ] += r;
      output.points[0].histogram[ shift_dim + 405 ] += r_;
      output.points[0].histogram[ shift_dim + 414 ] += g;
      output.points[0].histogram[ shift_dim + 423 ] += g_;
      output.points[0].histogram[ shift_dim + 432 ] += b;
      output.points[0].histogram[ shift_dim + 441 ] += b_;
    }
    break;
  case 1:
    if( center_bin_r ){
      output.points[0].histogram[ shift_dim +   7 ] += r;
      output.points[0].histogram[ shift_dim +  16 ] += r_;
      output.points[0].histogram[ shift_dim +  25 ] += g;
      output.points[0].histogram[ shift_dim +  34 ] += g_;
      output.points[0].histogram[ shift_dim +  43 ] += b;
      output.points[0].histogram[ shift_dim +  52 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim +  85 ] += r;
      output.points[0].histogram[ shift_dim +  94 ] += r_;
      output.points[0].histogram[ shift_dim + 103 ] += g;
      output.points[0].histogram[ shift_dim + 112 ] += g_;
      output.points[0].histogram[ shift_dim + 121 ] += b;
      output.points[0].histogram[ shift_dim + 130 ] += b_;
    }
    if( center_bin_g ){
      output.points[0].histogram[ shift_dim + 163 ] += r;
      output.points[0].histogram[ shift_dim + 172 ] += r_;
      output.points[0].histogram[ shift_dim + 181 ] += g;
      output.points[0].histogram[ shift_dim + 190 ] += g_;
      output.points[0].histogram[ shift_dim + 199 ] += b;
      output.points[0].histogram[ shift_dim + 208 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 241 ] += r;
      output.points[0].histogram[ shift_dim + 250 ] += r_;
      output.points[0].histogram[ shift_dim + 259 ] += g;
      output.points[0].histogram[ shift_dim + 268 ] += g_;
      output.points[0].histogram[ shift_dim + 277 ] += b;
      output.points[0].histogram[ shift_dim + 286 ] += b_;
    }
    if( center_bin_b ){
      output.points[0].histogram[ shift_dim + 319 ] += r;
      output.points[0].histogram[ shift_dim + 328 ] += r_;
      output.points[0].histogram[ shift_dim + 337 ] += g;
      output.points[0].histogram[ shift_dim + 346 ] += g_;
      output.points[0].histogram[ shift_dim + 355 ] += b;
      output.points[0].histogram[ shift_dim + 364 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 397 ] += r;
      output.points[0].histogram[ shift_dim + 406 ] += r_;
      output.points[0].histogram[ shift_dim + 415 ] += g;
      output.points[0].histogram[ shift_dim + 424 ] += g_;
      output.points[0].histogram[ shift_dim + 433 ] += b;
      output.points[0].histogram[ shift_dim + 442 ] += b_;
    }
    break;
  case 2:
    if( center_bin_r ){
      output.points[0].histogram[ shift_dim +   8 ] += r;
      output.points[0].histogram[ shift_dim +  17 ] += r_;
      output.points[0].histogram[ shift_dim +  26 ] += g;
      output.points[0].histogram[ shift_dim +  35 ] += g_;
      output.points[0].histogram[ shift_dim +  44 ] += b;
      output.points[0].histogram[ shift_dim +  53 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim +  86 ] += r;
      output.points[0].histogram[ shift_dim +  95 ] += r_;
      output.points[0].histogram[ shift_dim + 104 ] += g;
      output.points[0].histogram[ shift_dim + 113 ] += g_;
      output.points[0].histogram[ shift_dim + 122 ] += b;
      output.points[0].histogram[ shift_dim + 131 ] += b_;
    }
    if( center_bin_g ){
      output.points[0].histogram[ shift_dim + 164 ] += r;
      output.points[0].histogram[ shift_dim + 173 ] += r_;
      output.points[0].histogram[ shift_dim + 182 ] += g;
      output.points[0].histogram[ shift_dim + 191 ] += g_;
      output.points[0].histogram[ shift_dim + 200 ] += b;
      output.points[0].histogram[ shift_dim + 209 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 242 ] += r;
      output.points[0].histogram[ shift_dim + 251 ] += r_;
      output.points[0].histogram[ shift_dim + 260 ] += g;
      output.points[0].histogram[ shift_dim + 269 ] += g_;
      output.points[0].histogram[ shift_dim + 278 ] += b;
      output.points[0].histogram[ shift_dim + 287 ] += b_;
    }
    if( center_bin_b){
      output.points[0].histogram[ shift_dim + 320 ] += r;
      output.points[0].histogram[ shift_dim + 329 ] += r_;
      output.points[0].histogram[ shift_dim + 338 ] += g;
      output.points[0].histogram[ shift_dim + 347 ] += g_;
      output.points[0].histogram[ shift_dim + 356 ] += b;
      output.points[0].histogram[ shift_dim + 365 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 398 ] += r;
      output.points[0].histogram[ shift_dim + 407 ] += r_;
      output.points[0].histogram[ shift_dim + 416 ] += g;
      output.points[0].histogram[ shift_dim + 425 ] += g_;
      output.points[0].histogram[ shift_dim + 434 ] += b;
      output.points[0].histogram[ shift_dim + 443 ] += b_;
    }

    break;
  case 3:
    if( center_bin_r ){
      output.points[0].histogram[ shift_dim +   9 ] += r;
      output.points[0].histogram[ shift_dim +  18 ] += r_;
      output.points[0].histogram[ shift_dim +  27 ] += g;
      output.points[0].histogram[ shift_dim +  36 ] += g_;
      output.points[0].histogram[ shift_dim +  45 ] += b;
      output.points[0].histogram[ shift_dim +  54 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim +  87 ] += r;
      output.points[0].histogram[ shift_dim +  96 ] += r_;
      output.points[0].histogram[ shift_dim + 105 ] += g;
      output.points[0].histogram[ shift_dim + 114 ] += g_;
      output.points[0].histogram[ shift_dim + 123 ] += b;
      output.points[0].histogram[ shift_dim + 132 ] += b_;
    }
    if( center_bin_g ){
      output.points[0].histogram[ shift_dim + 165 ] += r;
      output.points[0].histogram[ shift_dim + 174 ] += r_;
      output.points[0].histogram[ shift_dim + 183 ] += g;
      output.points[0].histogram[ shift_dim + 192 ] += g_;
      output.points[0].histogram[ shift_dim + 201 ] += b;
      output.points[0].histogram[ shift_dim + 210 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 243 ] += r;
      output.points[0].histogram[ shift_dim + 252 ] += r_;
      output.points[0].histogram[ shift_dim + 261 ] += g;
      output.points[0].histogram[ shift_dim + 270 ] += g_;
      output.points[0].histogram[ shift_dim + 279 ] += b;
      output.points[0].histogram[ shift_dim + 288 ] += b_;
    }
    if( center_bin_b ){
      output.points[0].histogram[ shift_dim + 321 ] += r;
      output.points[0].histogram[ shift_dim + 330 ] += r_;
      output.points[0].histogram[ shift_dim + 339 ] += g;
      output.points[0].histogram[ shift_dim + 348 ] += g_;
      output.points[0].histogram[ shift_dim + 357 ] += b;
      output.points[0].histogram[ shift_dim + 366 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 399 ] += r;
      output.points[0].histogram[ shift_dim + 408 ] += r_;
      output.points[0].histogram[ shift_dim + 417 ] += g;
      output.points[0].histogram[ shift_dim + 426 ] += g_;
      output.points[0].histogram[ shift_dim + 435 ] += b;
      output.points[0].histogram[ shift_dim + 444 ] += b_;
    }
    break;
  case 4:
    if( center_bin_r ){
      output.points[0].histogram[ shift_dim +  10 ] += r;
      output.points[0].histogram[ shift_dim +  19 ] += r_;
      output.points[0].histogram[ shift_dim +  28 ] += g;
      output.points[0].histogram[ shift_dim +  37 ] += g_;
      output.points[0].histogram[ shift_dim +  46 ] += b;
      output.points[0].histogram[ shift_dim +  55 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim +  88 ] += r;
      output.points[0].histogram[ shift_dim +  97 ] += r_;
      output.points[0].histogram[ shift_dim + 106 ] += g;
      output.points[0].histogram[ shift_dim + 115 ] += g_;
      output.points[0].histogram[ shift_dim + 124 ] += b;
      output.points[0].histogram[ shift_dim + 133 ] += b_;
    }
    if( center_bin_g ){
      output.points[0].histogram[ shift_dim + 166 ] += r;
      output.points[0].histogram[ shift_dim + 175 ] += r_;
      output.points[0].histogram[ shift_dim + 184 ] += g;
      output.points[0].histogram[ shift_dim + 193 ] += g_;
      output.points[0].histogram[ shift_dim + 202 ] += b;
      output.points[0].histogram[ shift_dim + 211 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 244 ] += r;
      output.points[0].histogram[ shift_dim + 253 ] += r_;
      output.points[0].histogram[ shift_dim + 262 ] += g;
      output.points[0].histogram[ shift_dim + 271 ] += g_;
      output.points[0].histogram[ shift_dim + 280 ] += b;
      output.points[0].histogram[ shift_dim + 289 ] += b_;
    }
    if( center_bin_b ){
      output.points[0].histogram[ shift_dim + 322 ] += r;
      output.points[0].histogram[ shift_dim + 331 ] += r_;
      output.points[0].histogram[ shift_dim + 340 ] += g;
      output.points[0].histogram[ shift_dim + 349 ] += g_;
      output.points[0].histogram[ shift_dim + 358 ] += b;
      output.points[0].histogram[ shift_dim + 367 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 400 ] += r;
      output.points[0].histogram[ shift_dim + 409 ] += r_;
      output.points[0].histogram[ shift_dim + 418 ] += g;
      output.points[0].histogram[ shift_dim + 427 ] += g_;
      output.points[0].histogram[ shift_dim + 436 ] += b;
      output.points[0].histogram[ shift_dim + 445 ] += b_;
    }
    break;
  case 5:
    if( center_bin_r ){
      output.points[0].histogram[ shift_dim +  11 ] += r;
      output.points[0].histogram[ shift_dim +  20 ] += r_;
      output.points[0].histogram[ shift_dim +  29 ] += g;
      output.points[0].histogram[ shift_dim +  38 ] += g_;
      output.points[0].histogram[ shift_dim +  47 ] += b;
      output.points[0].histogram[ shift_dim +  56 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim +  89 ] += r;
      output.points[0].histogram[ shift_dim +  98 ] += r_;
      output.points[0].histogram[ shift_dim + 107 ] += g;
      output.points[0].histogram[ shift_dim + 116 ] += g_;
      output.points[0].histogram[ shift_dim + 125 ] += b;
      output.points[0].histogram[ shift_dim + 134 ] += b_;
    }
    if( center_bin_g ){
      output.points[0].histogram[ shift_dim + 167 ] += r;
      output.points[0].histogram[ shift_dim + 176 ] += r_;
      output.points[0].histogram[ shift_dim + 185 ] += g;
      output.points[0].histogram[ shift_dim + 194 ] += g_;
      output.points[0].histogram[ shift_dim + 203 ] += b;
      output.points[0].histogram[ shift_dim + 212 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 245 ] += r;
      output.points[0].histogram[ shift_dim + 254 ] += r_;
      output.points[0].histogram[ shift_dim + 263 ] += g;
      output.points[0].histogram[ shift_dim + 272 ] += g_;
      output.points[0].histogram[ shift_dim + 281 ] += b;
      output.points[0].histogram[ shift_dim + 290 ] += b_;
    }
    if( center_bin_b ){
      output.points[0].histogram[ shift_dim + 323 ] += r;
      output.points[0].histogram[ shift_dim + 332 ] += r_;
      output.points[0].histogram[ shift_dim + 341 ] += g;
      output.points[0].histogram[ shift_dim + 350 ] += g_;
      output.points[0].histogram[ shift_dim + 359 ] += b;
      output.points[0].histogram[ shift_dim + 368 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 401 ] += r;
      output.points[0].histogram[ shift_dim + 410 ] += r_;
      output.points[0].histogram[ shift_dim + 419 ] += g;
      output.points[0].histogram[ shift_dim + 428 ] += g_;
      output.points[0].histogram[ shift_dim + 437 ] += b;
      output.points[0].histogram[ shift_dim + 446 ] += b_;
    }
    break;
  case 6:
    if( center_bin_r ){
      output.points[0].histogram[ shift_dim +  12 ] += r;
      output.points[0].histogram[ shift_dim +  21 ] += r_;
      output.points[0].histogram[ shift_dim +  30 ] += g;
      output.points[0].histogram[ shift_dim +  39 ] += g_;
      output.points[0].histogram[ shift_dim +  48 ] += b;
      output.points[0].histogram[ shift_dim +  57 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim +  90 ] += r;
      output.points[0].histogram[ shift_dim +  99 ] += r_;
      output.points[0].histogram[ shift_dim + 108 ] += g;
      output.points[0].histogram[ shift_dim + 117 ] += g_;
      output.points[0].histogram[ shift_dim + 126 ] += b;
      output.points[0].histogram[ shift_dim + 135 ] += b_;
    }
    if( center_bin_g ){
      output.points[0].histogram[ shift_dim + 168 ] += r;
      output.points[0].histogram[ shift_dim + 177 ] += r_;
      output.points[0].histogram[ shift_dim + 186 ] += g;
      output.points[0].histogram[ shift_dim + 195 ] += g_;
      output.points[0].histogram[ shift_dim + 204 ] += b;
      output.points[0].histogram[ shift_dim + 213 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 246 ] += r;
      output.points[0].histogram[ shift_dim + 255 ] += r_;
      output.points[0].histogram[ shift_dim + 264 ] += g;
      output.points[0].histogram[ shift_dim + 273 ] += g_;
      output.points[0].histogram[ shift_dim + 282 ] += b;
      output.points[0].histogram[ shift_dim + 291 ] += b_;
    }
    if( center_bin_b ){
      output.points[0].histogram[ shift_dim + 324 ] += r;
      output.points[0].histogram[ shift_dim + 333 ] += r_;
      output.points[0].histogram[ shift_dim + 342 ] += g;
      output.points[0].histogram[ shift_dim + 351 ] += g_;
      output.points[0].histogram[ shift_dim + 360 ] += b;
      output.points[0].histogram[ shift_dim + 369 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 402 ] += r;
      output.points[0].histogram[ shift_dim + 411 ] += r_;
      output.points[0].histogram[ shift_dim + 420 ] += g;
      output.points[0].histogram[ shift_dim + 429 ] += g_;
      output.points[0].histogram[ shift_dim + 438 ] += b;
      output.points[0].histogram[ shift_dim + 447 ] += b_;
    }
    break;
  case 7:
    if( center_bin_r ){
      output.points[0].histogram[ shift_dim +  13 ] += r;
      output.points[0].histogram[ shift_dim +  22 ] += r_;
      output.points[0].histogram[ shift_dim +  31 ] += g;
      output.points[0].histogram[ shift_dim +  40 ] += g_;
      output.points[0].histogram[ shift_dim +  49 ] += b;
      output.points[0].histogram[ shift_dim +  58 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim +  91 ] += r;
      output.points[0].histogram[ shift_dim + 100 ] += r_;
      output.points[0].histogram[ shift_dim + 109 ] += g;
      output.points[0].histogram[ shift_dim + 118 ] += g_;
      output.points[0].histogram[ shift_dim + 127 ] += b;
      output.points[0].histogram[ shift_dim + 136 ] += b_;
    }
    if( center_bin_g ){
      output.points[0].histogram[ shift_dim + 169 ] += r;
      output.points[0].histogram[ shift_dim + 178 ] += r_;
      output.points[0].histogram[ shift_dim + 187 ] += g;
      output.points[0].histogram[ shift_dim + 196 ] += g_;
      output.points[0].histogram[ shift_dim + 205 ] += b;
      output.points[0].histogram[ shift_dim + 214 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 247 ] += r;
      output.points[0].histogram[ shift_dim + 256 ] += r_;
      output.points[0].histogram[ shift_dim + 265 ] += g;
      output.points[0].histogram[ shift_dim + 274 ] += g_;
      output.points[0].histogram[ shift_dim + 283 ] += b;
      output.points[0].histogram[ shift_dim + 292 ] += b_;
    }
    if( center_bin_b ){
      output.points[0].histogram[ shift_dim + 325 ] += r;
      output.points[0].histogram[ shift_dim + 334 ] += r_;
      output.points[0].histogram[ shift_dim + 343 ] += g;
      output.points[0].histogram[ shift_dim + 352 ] += g_;
      output.points[0].histogram[ shift_dim + 361 ] += b;
      output.points[0].histogram[ shift_dim + 370 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 403 ] += r;
      output.points[0].histogram[ shift_dim + 412 ] += r_;
      output.points[0].histogram[ shift_dim + 421 ] += g;
      output.points[0].histogram[ shift_dim + 430 ] += g_;
      output.points[0].histogram[ shift_dim + 439 ] += b;
      output.points[0].histogram[ shift_dim + 448 ] += b_;
    }
    break;
  case 8:
    if( center_bin_r ){
      output.points[0].histogram[ shift_dim +  14 ] += r;
      output.points[0].histogram[ shift_dim +  23 ] += r_;
      output.points[0].histogram[ shift_dim +  32 ] += g;
      output.points[0].histogram[ shift_dim +  41 ] += g_;
      output.points[0].histogram[ shift_dim +  50 ] += b;
      output.points[0].histogram[ shift_dim +  59 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim +  92 ] += r;
      output.points[0].histogram[ shift_dim + 101 ] += r_;
      output.points[0].histogram[ shift_dim + 110 ] += g;
      output.points[0].histogram[ shift_dim + 119 ] += g_;
      output.points[0].histogram[ shift_dim + 128 ] += b;
      output.points[0].histogram[ shift_dim + 137 ] += b_;
    }
    if( center_bin_g ){
      output.points[0].histogram[ shift_dim + 170 ] += r;
      output.points[0].histogram[ shift_dim + 179 ] += r_;
      output.points[0].histogram[ shift_dim + 188 ] += g;
      output.points[0].histogram[ shift_dim + 197 ] += g_;
      output.points[0].histogram[ shift_dim + 206 ] += b;
      output.points[0].histogram[ shift_dim + 215 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 248 ] += r;
      output.points[0].histogram[ shift_dim + 257 ] += r_;
      output.points[0].histogram[ shift_dim + 266 ] += g;
      output.points[0].histogram[ shift_dim + 275 ] += g_;
      output.points[0].histogram[ shift_dim + 284 ] += b;
      output.points[0].histogram[ shift_dim + 293 ] += b_;
    }
    if( center_bin_b ){
      output.points[0].histogram[ shift_dim + 326 ] += r;
      output.points[0].histogram[ shift_dim + 335 ] += r_;
      output.points[0].histogram[ shift_dim + 344 ] += g;
      output.points[0].histogram[ shift_dim + 353 ] += g_;
      output.points[0].histogram[ shift_dim + 362 ] += b;
      output.points[0].histogram[ shift_dim + 371 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 404 ] += r;
      output.points[0].histogram[ shift_dim + 413 ] += r_;
      output.points[0].histogram[ shift_dim + 422 ] += g;
      output.points[0].histogram[ shift_dim + 431 ] += g_;
      output.points[0].histogram[ shift_dim + 440 ] += b;
      output.points[0].histogram[ shift_dim + 449 ] += b_;
    }
    break;
  case 9:
    if( center_bin_r ){
      output.points[0].histogram[ shift_dim +  60 ] += r;
      output.points[0].histogram[ shift_dim +  64 ] += r_;
      output.points[0].histogram[ shift_dim +  68 ] += g;
      output.points[0].histogram[ shift_dim +  72 ] += g_;
      output.points[0].histogram[ shift_dim +  76 ] += b;
      output.points[0].histogram[ shift_dim +  80 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 138 ] += r;
      output.points[0].histogram[ shift_dim + 142 ] += r_;
      output.points[0].histogram[ shift_dim + 146 ] += g;
      output.points[0].histogram[ shift_dim + 150 ] += g_;
      output.points[0].histogram[ shift_dim + 154 ] += b;
      output.points[0].histogram[ shift_dim + 158 ] += b_;
    }
    if( center_bin_g ){
      output.points[0].histogram[ shift_dim + 216 ] += r;
      output.points[0].histogram[ shift_dim + 220 ] += r_;
      output.points[0].histogram[ shift_dim + 224 ] += g;
      output.points[0].histogram[ shift_dim + 228 ] += g_;
      output.points[0].histogram[ shift_dim + 232 ] += b;
      output.points[0].histogram[ shift_dim + 236 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 294 ] += r;
      output.points[0].histogram[ shift_dim + 298 ] += r_;
      output.points[0].histogram[ shift_dim + 302 ] += g;
      output.points[0].histogram[ shift_dim + 306 ] += g_;
      output.points[0].histogram[ shift_dim + 310 ] += b;
      output.points[0].histogram[ shift_dim + 314 ] += b_;
    }
    if( center_bin_b ){
      output.points[0].histogram[ shift_dim + 372 ] += r;
      output.points[0].histogram[ shift_dim + 376 ] += r_;
      output.points[0].histogram[ shift_dim + 380 ] += g;
      output.points[0].histogram[ shift_dim + 384 ] += g_;
      output.points[0].histogram[ shift_dim + 388 ] += b;
      output.points[0].histogram[ shift_dim + 392 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 450 ] += r;
      output.points[0].histogram[ shift_dim + 454 ] += r_;
      output.points[0].histogram[ shift_dim + 458 ] += g;
      output.points[0].histogram[ shift_dim + 462 ] += g_;
      output.points[0].histogram[ shift_dim + 466 ] += b;
      output.points[0].histogram[ shift_dim + 470 ] += b_;
    }
    break;
  case 10:
    if( center_bin_r ){
      output.points[0].histogram[ shift_dim +  61 ] += r;
      output.points[0].histogram[ shift_dim +  65 ] += r_;
      output.points[0].histogram[ shift_dim +  69 ] += g;
      output.points[0].histogram[ shift_dim +  73 ] += g_;
      output.points[0].histogram[ shift_dim +  77 ] += b;
      output.points[0].histogram[ shift_dim +  81 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 139 ] += r;
      output.points[0].histogram[ shift_dim + 143 ] += r_;
      output.points[0].histogram[ shift_dim + 147 ] += g;
      output.points[0].histogram[ shift_dim + 151 ] += g_;
      output.points[0].histogram[ shift_dim + 155 ] += b;
      output.points[0].histogram[ shift_dim + 159 ] += b_;
    }
    if( center_bin_g ){
      output.points[0].histogram[ shift_dim + 217 ] += r;
      output.points[0].histogram[ shift_dim + 221 ] += r_;
      output.points[0].histogram[ shift_dim + 225 ] += g;
      output.points[0].histogram[ shift_dim + 229 ] += g_;
      output.points[0].histogram[ shift_dim + 233 ] += b;
      output.points[0].histogram[ shift_dim + 237 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 295 ] += r;
      output.points[0].histogram[ shift_dim + 299 ] += r_;
      output.points[0].histogram[ shift_dim + 303 ] += g;
      output.points[0].histogram[ shift_dim + 307 ] += g_;
      output.points[0].histogram[ shift_dim + 311 ] += b;
      output.points[0].histogram[ shift_dim + 315 ] += b_;
    }
    if( center_bin_b ){
      output.points[0].histogram[ shift_dim + 373 ] += r;
      output.points[0].histogram[ shift_dim + 377 ] += r_;
      output.points[0].histogram[ shift_dim + 381 ] += g;
      output.points[0].histogram[ shift_dim + 385 ] += g_;
      output.points[0].histogram[ shift_dim + 389 ] += b;
      output.points[0].histogram[ shift_dim + 393 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 451 ] += r;
      output.points[0].histogram[ shift_dim + 455 ] += r_;
      output.points[0].histogram[ shift_dim + 459 ] += g;
      output.points[0].histogram[ shift_dim + 463 ] += g_;
      output.points[0].histogram[ shift_dim + 467 ] += b;
      output.points[0].histogram[ shift_dim + 471 ] += b_;
    }
    break;
  case 11:
    if( center_bin_r ){
      output.points[0].histogram[ shift_dim +  62 ] += r;
      output.points[0].histogram[ shift_dim +  66 ] += r_;
      output.points[0].histogram[ shift_dim +  70 ] += g;
      output.points[0].histogram[ shift_dim +  74 ] += g_;
      output.points[0].histogram[ shift_dim +  78 ] += b;
      output.points[0].histogram[ shift_dim +  82 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 140 ] += r;
      output.points[0].histogram[ shift_dim + 144 ] += r_;
      output.points[0].histogram[ shift_dim + 148 ] += g;
      output.points[0].histogram[ shift_dim + 152 ] += g_;
      output.points[0].histogram[ shift_dim + 156 ] += b;
      output.points[0].histogram[ shift_dim + 160 ] += b_;
    }
    if( center_bin_g ){
      output.points[0].histogram[ shift_dim + 218 ] += r;
      output.points[0].histogram[ shift_dim + 222 ] += r_;
      output.points[0].histogram[ shift_dim + 226 ] += g;
      output.points[0].histogram[ shift_dim + 230 ] += g_;
      output.points[0].histogram[ shift_dim + 234 ] += b;
      output.points[0].histogram[ shift_dim + 238 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 296 ] += r;
      output.points[0].histogram[ shift_dim + 300 ] += r_;
      output.points[0].histogram[ shift_dim + 304 ] += g;
      output.points[0].histogram[ shift_dim + 308 ] += g_;
      output.points[0].histogram[ shift_dim + 312 ] += b;
      output.points[0].histogram[ shift_dim + 316 ] += b_;
    }
    if( center_bin_b ){
      output.points[0].histogram[ shift_dim + 374 ] += r;
      output.points[0].histogram[ shift_dim + 378 ] += r_;
      output.points[0].histogram[ shift_dim + 382 ] += g;
      output.points[0].histogram[ shift_dim + 386 ] += g_;
      output.points[0].histogram[ shift_dim + 390 ] += b;
      output.points[0].histogram[ shift_dim + 394 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 452 ] += r;
      output.points[0].histogram[ shift_dim + 456 ] += r_;
      output.points[0].histogram[ shift_dim + 460 ] += g;
      output.points[0].histogram[ shift_dim + 464 ] += g_;
      output.points[0].histogram[ shift_dim + 468 ] += b;
      output.points[0].histogram[ shift_dim + 472 ] += b_;
    }
    break;
  case 12:
    if( center_bin_r ){
      output.points[0].histogram[ shift_dim +  63 ] += r;
      output.points[0].histogram[ shift_dim +  67 ] += r_;
      output.points[0].histogram[ shift_dim +  71 ] += g;
      output.points[0].histogram[ shift_dim +  75 ] += g_;
      output.points[0].histogram[ shift_dim +  79 ] += b;
      output.points[0].histogram[ shift_dim +  83 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 141 ] += r;
      output.points[0].histogram[ shift_dim + 145 ] += r_;
      output.points[0].histogram[ shift_dim + 149 ] += g;
      output.points[0].histogram[ shift_dim + 153 ] += g_;
      output.points[0].histogram[ shift_dim + 157 ] += b;
      output.points[0].histogram[ shift_dim + 161 ] += b_;
    }
    if( center_bin_g ){
      output.points[0].histogram[ shift_dim + 219 ] += r;
      output.points[0].histogram[ shift_dim + 223 ] += r_;
      output.points[0].histogram[ shift_dim + 227 ] += g;
      output.points[0].histogram[ shift_dim + 231 ] += g_;
      output.points[0].histogram[ shift_dim + 235 ] += b;
      output.points[0].histogram[ shift_dim + 239 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 297 ] += r;
      output.points[0].histogram[ shift_dim + 301 ] += r_;
      output.points[0].histogram[ shift_dim + 305 ] += g;
      output.points[0].histogram[ shift_dim + 309 ] += g_;
      output.points[0].histogram[ shift_dim + 313 ] += b;
      output.points[0].histogram[ shift_dim + 317 ] += b_;
    }
    if( center_bin_b ){
      output.points[0].histogram[ shift_dim + 375 ] += r;
      output.points[0].histogram[ shift_dim + 379 ] += r_;
      output.points[0].histogram[ shift_dim + 383 ] += g;
      output.points[0].histogram[ shift_dim + 387 ] += g_;
      output.points[0].histogram[ shift_dim + 391 ] += b;
      output.points[0].histogram[ shift_dim + 395 ] += b_;
    }
    else{
      output.points[0].histogram[ shift_dim + 453 ] += r;
      output.points[0].histogram[ shift_dim + 457 ] += r_;
      output.points[0].histogram[ shift_dim + 461 ] += g;
      output.points[0].histogram[ shift_dim + 465 ] += g_;
      output.points[0].histogram[ shift_dim + 469 ] += b;
      output.points[0].histogram[ shift_dim + 473 ] += b_;
    }
    break;
    //   case 13: // itself
    //     break;
    //   case 14: // 14 - 26: redundant
    //     break;
  default:
    break;
  }

}

inline void
pcl::ColorCHLACEstimation::computeColorCHLAC ( const pcl::PointCloud<PointXYZRGB> &cloud,
					       PointCloudOut &output, const int center_idx )
{
  color = *reinterpret_cast<const int*>(&(cloud.points[center_idx].rgb));
  center_r = (0xff0000 & color) >> 16;
  center_g = (0x00ff00 & color) >> 8;
  center_b =  0x0000ff & color;
  center_r_ = reverse( center_r );
  center_g_ = reverse( center_g );
  center_b_ = reverse( center_b );
  center_bin_r = binarize_r( center_r );
  center_bin_g = binarize_g( center_g );
  center_bin_b = binarize_b( center_b );
  addColorCHLAC_0( output );
  addColorCHLAC_0_bin( output );

//   vector<Eigen3::Vector3i> directions;                                          
//   directions.push_back (Eigen3::Vector3i (0, 0, 1)); 
//   std::vector<int> neighbors = grid.getNeighborCentroidIndices (cloud.points[center_idx], directions);
  std::vector<int> neighbors = grid.getNeighborCentroidIndices (cloud.points[center_idx], relative_coordinates);

  for (int i = 0; i < 12; ++i){
    // Check if the point is invalid
    if ( neighbors[i]!=-1 ){
      color = *reinterpret_cast<const int*>(&(cloud.points[neighbors[i]].rgb));
      const int r = (0xff0000 & color) >> 16;
      const int g = (0x00ff00 & color) >> 8;
      const int b =  0x0000ff & color;
    
      addColorCHLAC_1 ( output, i, r, g, b );
      addColorCHLAC_1_bin ( output, i, binarize_r(r), binarize_g(g), binarize_b(b) );
    }
  }
}

inline void
pcl::ColorCHLACEstimation::normalizeColorCHLAC ( PointCloudOut &output )
{
  for(int i=0; i<6; ++i)
    output.points[0].histogram[ i ] /= 765.0;
  for(int i=6; i<495; ++i)
    output.points[0].histogram[ i ] /= 585225.0;
  for(int i=495; i<501; ++i)
    output.points[0].histogram[ i ] /= 3.0;
  for(int i=501; i<981; ++i)
    output.points[0].histogram[ i ] /= 9.0;
}

inline void
pcl::ColorCHLACEstimation::computeFeature (PointCloudOut &output)
{
  if( (color_thR<0)||(color_thG<0)||(color_thB<0) ){
    std::cerr << "Invalid color_threshold: " << color_thR << " " << color_thG << " " << color_thB << endl;
    return;
  }

  // We only output _1_ signature
  output.points.resize (1);
  output.width = 1;
  output.height = 1;

  // initialize histogram
  for( int t=0; t<981; t++ )
    output.points[0].histogram[t] = 0;

  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
    computeColorCHLAC (*surface_, output, (*indices_)[idx] );
  normalizeColorCHLAC( output );
}
