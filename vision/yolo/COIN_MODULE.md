# COIN Module Notes

이 문서는 이 저장소에 있는 현재 COIN 구현을 설명한다.
일반적인 설계 메모가 아니라, 지금 코드가 실제로 무엇을 하는지,
각 부분이 왜 존재하는지, 어디에서 다른 설계를 시도해볼 수 있는지를
정리하는 것이 목적이다.

관련 코드:

- `vision/yolo/coin_pose.py`
- `vision/yolo/train_coin_pose.py`

## 1. 목표

COIN은 YOLO pose backbone 주변에 붙는 가벼운 플러그인으로,
backbone feature를 다음 상황에 더 강인하게 만들기 위한 구조다.

- illumination shift
- partial occlusion
- visible / hidden local evidence가 섞인 경우

현재 설계의 핵심 아이디어는 세 가지다.

1. 입력 이미지에서 illumination embedding을 추론한다
2. 그 embedding으로 backbone의 일부 intermediate feature를 modulate한다
3. multi-scale context와 visibility-weighted reconstruction branch를 사용해
   YOLO head에 들어가기 전 feature를 보정한다

이 저장소에서 COIN은 다음 YOLO feature map들에 부착된다.

- layer indices: `TARGET_LAYERS = [4, 6, 10]`
- channels: `[128, 128, 256]`

## 2. 전체 데이터 흐름

증강 입력 이미지 `I_aug`에 대해서:

```text
I_aug
  -> illumination encoder -> Z
  -> soft router          -> P
  -> 3개 YOLO feature map에 FiLM modulation
  -> context pyramid
  -> visibility + reconstruction
  -> refined feature maps
  -> YOLO pose head
```

Phase 1에서 clean reference image `I_clean`이 있으면:

```text
I_clean
  -> frozen teacher or frozen YOLO backbone
  -> clean target features / clean pose logits

I_clean
  -> COIN path again
  -> gate regularization + clean visibility supervision
```

즉 Phase 1에서 clean image는 두 가지 용도로 쓰인다.

- matching을 위한 clean target path
- gate와 visibility를 regularize하기 위한 clean COIN path

## 3. 모듈 구성

### 3.1 Illumination Encoder

코드:

- `SpatiallyVariantIlluminationEncoder`

세 번의 stride-2 convolution을 적용한다:

```text
Z = E(I_aug),  Z in R^(B x 64 x H/8 x W/8)
```

해석:

- `Z`는 단일 global vector가 아니라 spatial illumination descriptor다
- 밝음 / 어두움 / gradient lighting change를 위치별로 다르게 표현할 수 있다

설계 계보:

- appearance-conditioned representation
- illumination disentanglement 아이디어

### 3.2 Soft Expert Router

코드:

- `SpatiallyAwareSoftRouter`

router는 위치별 soft expert weight를 예측한다:

```text
P = softmax(R(Z), dim=expert)
P in R^(B x K x H/8 x W/8),  K = num_experts = 3
```

해석:

- 각 spatial location이 expert mixture를 선택한다
- 하나의 global condition vector보다, spatial mixture-of-experts /
  dynamic convolution style routing에 더 가깝다

### 3.3 Multi-Scale RSV-FiLM

코드:

- `MultiScaleRSVFiLM`

선택된 backbone feature map `F_s`의 각 scale `s`에 대해, COIN은
`Z`와 `P`를 해당 feature resolution으로 upsample하고 expert별 affine term을 예측한다.

각 expert `k`에 대해:

```text
gamma_s,k = Conv1x1_s,k(Z_s)
beta_s,k  = Conv1x1_s,k(Z_s)
```

router가 expert를 섞는다:

```text
DeltaGamma_s = sum_k P_s,k * gamma_s,k
DeltaBeta_s  = sum_k P_s,k * beta_s,k
```

그 다음 scalar spatial gate를 적용한다:

```text
G_s = sigmoid(W_gate,s * Z_s + b_gate,s)
```

현재 초기화:

- gate weight = 0
- gate bias = `-0.5`
- 따라서 초기 gate 값은 대략 `sigmoid(-0.5) ~= 0.38`

최종 modulated feature:

```text
F_mod,s = F_s * (1 + G_s * DeltaGamma_s) + G_s * DeltaBeta_s
```

해석:

- FiLM-style affine conditioning이다
- gate가 modulation을 residual 형태로 만들고 suppress 가능하게 한다
- `gamma` / `beta`는 small init이므로 초기에는 거의 identity에 가깝고,
  이후 점진적으로 비자명한 modulation을 학습할 수 있다

### 3.4 Context Pyramid

코드:

- `COINContextPyramid`

이 블록은 선택된 세 scale의 modulated feature를 받아,
top-down 다음 bottom-up context path를 만든다:

```text
[P3, P4, P5]
  -> lateral projection
  -> top-down fusion
  -> bottom-up fusion
  -> output projection
  -> context-enhanced features
```

역할:

- 높은 수준의 semantic context를 아래 scale로 전달
- 낮은 수준의 localization detail을 위 scale로 전달
- 작은 occluded region이 자기 주변의 망가진 local evidence만 보고
  reconstruction되는 것을 줄임

### 3.5 Occlusion Context Recovery

코드:

- `OcclusionContextRecovery`

각 scale에 대해:

```text
F_fused,s = concat(F_mod,s, F_ctx,s)
V_s       = sigmoid(H_vis,s(F_fused,s))
R_s       = F_mod,s + H_rec,s(F_fused,s)
F_out,s   = V_s * F_mod,s + (1 - V_s) * R_s
```

여기서:

- `V_s`는 1-channel visibility map
- `R_s`는 reconstructed feature
- `F_out,s`는 downstream에서 실제로 사용되는 최종 feature

해석:

- `V`가 1에 가까우면 modulated original feature를 신뢰
- `V`가 0에 가까우면 reconstructed feature를 신뢰
- `V`가 0과 1 사이면 둘을 섞는다

이것이 현재 코드에서 "occlusion-aware"의 핵심 메커니즘이다.

## 4. Wrapper 동작

코드:

- `COINPose`
- `patch_yolo_with_coin()`

실행 모드는 두 가지다.

### 4.1 Phase 1 Wrapper Mode

`COINPose`는 YOLO model을 직접 감싼다.

Phase 1 동작:

- YOLO backbone은 frozen
- COIN module은 trainable
- clean reference feature는 frozen YOLO path에서 추출
- degraded image에는 COIN을 적용
- loss는 clean target에 대해 계산

`i_clean`이 제공되면 wrapper는 modulated clean pass를 한 번 더 돌려서
다음을 얻는다:

- `gate_raws_clean`
- `visibility_maps_clean`

이 값들은 `identity`, `vis_reg` 같은 regularizer에 사용된다.

### 4.2 Phase 2 Patched YOLO Mode

`patch_yolo_with_coin()`은 COIN module을 Ultralytics `PoseModel` 안에 주입하고
`_predict_once()`를 override한다.

현재 Phase 2 path:

```text
input
  -> coin_ie
  -> coin_sr
  -> coin_rsv
  -> coin_ctx
  -> coin_or
  -> YOLO pose head
```

중요한 구현 사실:

- Phase 2는 Phase 1의 전체 COIN loss를 다시 사용하지 않는다
- 현재 추가되는 것은 오직:
  - Phase 1 COIN weight에 대한 anchor regularization
  - `DeltaGamma`, `DeltaBeta`에 대한 TV regularization

즉 Phase 2는 full COIN retraining이 아니라,
가벼운 auxiliary prior를 얹은 task tuning이다.

## 5. Phase 1 학습 pair

코드:

- `PairedSSLDataset`

두 종류의 pair가 만들어진다.

### 5.1 Illumination Pair

```text
(bright reference, dark degraded, "illum")
```

illumination adaptation에 사용된다.

### 5.2 Occlusion Pair

```text
(clean reference, bright degraded, "occlu")
```

occlusion-aware reconstruction에 사용된다.

현재 샘플링 비율은 대략:

```text
illum : occlu = 2 : 1
```

즉 illumination adaptation이 occlusion reconstruction보다 oversample되어 있다.

## 6. Loss: 현재 수식 기준 정리

## 6.1 Feature SSL Loss

코드:

- `FeatureConsistencySSLLoss`

feature pair `(F_aug, F_clean)`에 대해:

```text
L_ssl = SmoothL1( normalize(F_aug), normalize(stopgrad(F_clean)) )
```

이 loss는 주로 illumination pair에서 사용되고,
occlusion pair에서는 visible branch term에도 사용된다.

## 6.2 Occlusion-Aware Consistency Loss

코드:

- `OcclusionAwareConsistencyLoss`

현재 구현:

```text
T         = normalize(stopgrad(F_clean))
DiffFeat  = SmoothL1( normalize(F_mod), T, reduction="none" )
DiffRecon = SmoothL1( normalize(R),     T, reduction="none" )

L_visible  = mean(DiffFeat  * V)
L_occluded = mean(DiffRecon * (1 - stopgrad(V)))
```

중요한 점:

- docstring은 대칭적인 visibility gradient를 설명하고 있다
- 하지만 현재 코드는 `L_occluded` 안에서 `V`를 detach한다

즉 reconstruction branch는 `(1 - V)` weighting으로 학습되지만,
`V`는 `DiffRecon` 쪽의 음의 gradient를 받지 않는다.

따라서 현재 visibility learning signal은 주로:

- `L_visible`
- `vis_gt`
- `vis_reg`
- `vis_illum`

에서 오며, 완전한 `DiffFeat - DiffRecon` 경쟁 구조는 아니다.

이 점은 현재 visibility map의 동작을 해석할 때 중요하다.

## 6.3 Histogram Matching Loss

코드:

- `KDESoftHistogramLoss`

각 feature tensor에 대해 Gaussian kernel 기반 soft histogram을 만들고,
이를 MSE로 맞춘다:

```text
L_hist = MSE( SoftHist(F_aug), SoftHist(stopgrad(F_clean)) )
```

목적:

- illumination change 아래에서 global feature distribution consistency를 유지

## 6.4 Illumination Smoothness Loss

코드:

- `IlluminationSmoothLoss`

illumination embedding `Z`에 대해:

```text
L_smooth = 0.5 * ( mean(|dx Z|) + mean(|dy Z|) )
```

목적:

- noisy하고 고주파적인 illumination code를 억제

## 6.5 TV Loss on FiLM Residuals

코드:

- `TotalVariationLoss`

`DeltaGamma`, `DeltaBeta`에 적용된다:

```text
L_tv = sum_s TV(DeltaGamma_s) + TV(DeltaBeta_s)
```

목적:

- spatial modulation을 부드럽게 유지
- checkerboard / 고주파 modulation artifact를 줄임

## 6.6 Identity Gate Regularization

코드:

- `CleanIdentityRegLoss`

clean-image gate logit에 대해:

```text
L_identity = mean_s mean( softplus(GateRaw_clean,s) )
```

`softplus(x)`를 최소화하면 `x`를 음수 방향으로 밀기 때문에,
이 regularizer는 clean input에서 gate를 더 작게 만드는 방향으로 작동한다.

해석:

- 이미지가 이미 clean하다면 modulation을 너무 강하게 하지 말자

## 6.7 Clean Visibility Regularization

코드:

- `VisibilityRegLoss`

clean-image visibility map에 대해:

```text
L_vis_reg = mean_s MSE(V_clean,s, 1)
```

목적:

- clean branch에서는 reconstruction branch보다 original feature path를
  신뢰하도록 유도

## 6.8 Illumination Visibility Regularization

이 항도 `VisibilityRegLoss`를 사용하지만 `illum` sample에만 적용된다:

```text
L_vis_illum = mean_s MSE(V_illum,s, 1)
```

해석:

- 순수 illumination change는 occlusion으로 취급하면 안 된다
- 따라서 illumination-only pair에서는 visibility를 높게 유지하도록 유도

## 6.9 Visibility GT Supervision

코드:

- `VisibilityGTLoss`
- `PairedSSLDataset._load_vis_gt()`

degraded image의 YOLO label은 keypoint visibility label을 가진다:

```text
0 -> 0.0
1 -> 0.5
2 -> 1.0
```

이 keypoint들은 먼저 입력 파이프라인과 동일한 letterboxed square image
좌표계로 투영되고, 그 뒤 각 visibility map scale에서 샘플링된다:

```text
gx = floor(norm_x * w_s)
gy = floor(norm_y * h_s)
pred = V_s[gy, gx]
L_vis_gt = mean( (pred - gt_vis)^2 )
```

목적:

- sparse labeled keypoint location에 visibility learning을 고정
- generic한 "V를 binary하게 만들어라"보다 pose task에 더 가까운 supervision 제공

## 6.10 KD Loss

코드:

- `pose_logits_aug` vs `pose_logits_clean`에 대한 recursive L1

```text
L_kd = L1( pose_logits_aug, stopgrad(pose_logits_clean) )
```

목적:

- degraded-image pose head output이 clean-image output에 가깝게 유지되도록 함

## 7. 현재 활성화된 Phase Weight

현재 `COINLoss.PHASE_WEIGHTS`에 설정된 활성 weight:

### Phase 1

```text
kd        = 0.3
ssl       = 1.0
occ       = 1.0
hist      = 1.0
smooth    = 0.3
tv        = 0.05
identity  = 0.2
vis_reg   = 0.05
vis_illum = 0.10
vis_gt    = 0.5
```

### Phase 2

```text
kd        = 1.0
ssl       = 0.5
occ       = 0.8
hist      = 0.3
smooth    = 0.1
tv        = 0.05
identity  = 0.1
vis_reg   = 0.02
vis_illum = 0.05
vis_gt    = 0.3
```

하지만 다시 강조하면:

- 이것은 `COINLoss` 안의 weight 정의다
- patched Ultralytics Phase 2 path는 실제로 full `COINLoss`를 재사용하지 않는다
- 실제 Phase 2는 task loss + anchor reg + TV auxiliary loss만 쓴다

## 8. 현재 비활성 또는 legacy 상태인 항목

다음 항목들은 source에는 남아 있지만, 현재 학습 설정에서는 active path가 아니다.

### 8.1 Orthogonality Loss

`ExpertOrthogonalityLoss`는 `coin_pose.py` 안에 남아 있지만,
현재 active phase weight나 total loss path에는 포함되지 않는다.

### 8.2 Visibility Sparsity / Binary Entropy

`VisibilitySparsityLoss`, `VisibilityBinaryEntropyLoss` 역시 source에는 있지만,
현재 active loss path에서는 사용되지 않는다.

이 점은 중요하다. 현재 visibility supervision은 ternary target을 사용하기 때문이다:

```text
visible  -> 1.0
partial  -> 0.5
hidden   -> 0.0
```

`vis_gt`가 중심 supervision이 된 뒤에는 binary push를 제거한 방향이 맞다.

## 9. 현재 메커니즘의 실용적 해석

COIN을 한 줄로 요약하면:

```text
COIN = illumination-conditioned feature modulation
     + multi-scale context sharing
     + visibility-weighted feature repair
```

좀 더 구체적으로 말하면:

- illumination encoder는 "지금 어떤 photometric condition인가?"를 말해준다
- FiLM path는 "이 조건에서 intermediate feature를 어떻게 이동시켜야 하는가?"를 담당한다
- context pyramid는 "주변 scale을 보면 이 영역이 원래 어떻게 생겼어야 하는가?"를 제공한다
- visibility head는 "여기서는 original feature를 믿을지, repaired feature를 믿을지"를 결정한다

이것이 COIN이 preprocessing baseline과 다른 이유다:

- preprocessing은 YOLO 앞에서 pixel을 수정한다
- COIN은 YOLO 내부에서 task feature를 image-dependent하게 수정한다

## 10. 현재 구조의 강점

- 기본 detector architecture를 크게 바꾸지 않는다
- 전체 network가 아니라 선택된 feature map만 modulate한다
- illumination adaptation과 occlusion repair를 명시적으로 분리한다
- task-adjacent supervision (`vis_gt`, KD, clean feature matching)을 사용한다
- deployment 비용이 MSRCR 같은 무거운 image-space preprocessing보다 낮다

## 11. 현재 구조의 한계

이 항목들은 이론적인 불만이 아니라, 현재 코드로부터 직접 나오는 한계다.

### 11.1 Visibility가 reconstruction error로부터 완전한 양방향 supervision을 받지 않는다

현재:

```text
L_occluded = mean(DiffRecon * (1 - stopgrad(V)))
```

이므로 visibility map은 "여기서는 reconstruction이 더 좋다"는 신호를
그 branch를 통해 직접 받지 못한다.

영향:

- recon은 학습된다
- visibility는 docstring이 암시하는 것보다 recon quality와 느슨하게 결합되어 있다

### 11.2 Phase 2는 visibility behavior를 명시적으로 보존하지 않는다

patched Phase 2 training은 다음만 유지한다:

- task loss
- anchor regularization
- TV regularization

하지만 다음은 유지하지 않는다:

- `vis_gt`
- `vis_reg`
- `vis_illum`
- `occ_loss`

따라서 task loss가 원하면 Phase 1에서 배운 visibility / reconstruction behavior가
Phase 2에서 drift할 수 있다.

### 11.3 Checkpoint selection이 일부 visibility objective를 무시한다

Phase 1 validation은 `core` metric을 다음을 제외하고 계산한다:

- `vis_reg`
- `vis_illum`
- `vis_gt`
- `total`

이 항들을 regularizer처럼 보는 관점에서는 합리적이지만,
반대로 말하면 best checkpoint selection이 visibility-map quality를
직접 최적화하지는 않는다는 뜻이기도 하다.

## 12. 시도해볼 수 있는 설계 변경 지점

아래 unchecked item들은 자연스러운 실험 후보들이다.

### 12.1 Illumination Encoding

- [ ] shallow CNN encoder를 더 가벼운 global-plus-local encoder로 교체
- [ ] explicit low-frequency image statistic 또는 HSV / log-intensity input 추가
- [ ] illumination feature를 scratch로 학습하지 말고 Retinex-style teacher에서 distill

### 12.2 Expert Routing

- [ ] soft full routing 대신 top-k routing 사용
- [ ] expert별 1x1 FiLM head를 CondConv / dynamic-conv style kernel로 대체
- [ ] runtime이 더 중요하면 expert 수를 3에서 2로 줄이기

### 12.3 FiLM Gate

- [ ] 1-channel spatial gate 대신 per-channel gate 시도
- [ ] gate bias를 다시 더 낮춰 stronger identity prior 적용
- [ ] `sigmoid(gate_raw)` 대신 `tanh(gate_raw)`나 bounded scale parameter 사용

### 12.4 Context Aggregation

- [ ] 현재 FPN+PAN style pyramid를 BiFPN으로 교체
- [ ] attention-based cross-scale fusion 추가
- [ ] bottom-up fusion을 제거하고 top-down only로 충분한지 확인

### 12.5 Visibility / Reconstruction Coupling

- [ ] `L_occluded`에서 `stopgrad(V)`를 제거해 symmetric competition 복원
- [ ] direct visibility scalar 대신 uncertainty-weighted / confidence-weighted blending 사용
- [ ] `concat(F_mod, F_ctx)`만이 아니라 `F_mod`와 `R - F_mod`를 같이 써서 visibility 예측

### 12.6 Visibility Supervision

- [ ] sparse keypoint뿐 아니라 occluder region에 대한 dense pseudo-label 추가
- [ ] self-occlusion과 foreign occlusion 구분
- [ ] partial GT가 없는 위치에만 binary prior를 다시 도입

### 12.7 Phase 2 Retention

- [ ] Phase 2에 약한 `vis_gt` auxiliary term 유지
- [ ] Phase 2에 약한 clean-image consistency branch 유지
- [ ] `_coin_cache`에 visibility map도 저장하고 task tuning 동안 regularize

### 12.8 Latency

- [ ] `coin_ctx`, `coin_or`를 제거한 FiLM-only ablation 테스트
- [ ] fixed gate를 사용하는 context-only ablation 테스트
- [ ] runtime-sensitive deployment를 위해 target layer를 3개에서 2개로 축소

## 13. 추천 최소 ablation 표

COIN이 실제로 무엇을 하고 있는지 깔끔하게 이해하려면,
최소한 다음 정도는 비교하는 것이 좋다.

1. plain YOLO
2. CLAHE 또는 MSRCR preprocessing baseline
3. FiLM-only COIN
4. FiLM + context
5. full COIN
6. `vis_gt` 없는 full COIN
7. symmetric `occ_loss` visibility gradient를 가진 full COIN

이 표는 다음 질문에 답하게 해준다:

- 성능 향상의 대부분이 photometric adaptation 때문인가?
- context sharing이 실제로 의미 있는 역할을 하는가?
- visibility / reconstruction branch가 정말 필요한가?
- visibility learning의 핵심이 `vis_gt`인가?

## 14. Reference Papers

이 논문들은 현재 구조의 주요 구성 요소와 가장 잘 맞닿아 있는 레퍼런스들이다.

- FiLM: Perez et al., "FiLM: Visual Reasoning with a General Conditioning Layer," AAAI 2018.
  Link: https://cdn.aaai.org/ojs/11671/11671-13-15199-1-2-20201228.pdf

- FPN: Lin et al., "Feature Pyramid Networks for Object Detection," CVPR 2017.
  Link: https://openaccess.thecvf.com/content_cvpr_2017/papers/Lin_Feature_Pyramid_Networks_CVPR_2017_paper.pdf

- PANet: Liu et al., "Path Aggregation Network for Instance Segmentation," CVPR 2018.
  Link: https://jiaya.me/file/papers/panet_cvpr18.pdf

- Dynamic convolution / input-dependent expert aggregation:
  Chen et al., "Dynamic Convolution: Attention Over Convolution Kernels," CVPR 2020.
  Link: https://openaccess.thecvf.com/content_CVPR_2020/html/Chen_Dynamic_Convolution_Attention_Over_Convolution_Kernels_CVPR_2020_paper.html

- Knowledge distillation:
  Hinton et al., "Distilling the Knowledge in a Neural Network," 2015.
  Link: https://www.cs.toronto.edu/~hinton/absps/distillation.pdf

- Appearance and transient-factor disentanglement:
  Martin-Brualla et al., "NeRF in the Wild: Neural Radiance Fields for Unconstrained Photo Collections," CVPR 2021.
  Link: https://openaccess.thecvf.com/content/CVPR2021/html/Martin-Brualla_NeRF_in_the_Wild_Neural_Radiance_Fields_for_Unconstrained_Photo_CVPR_2021_paper.html

- Uncertainty-driven distractor handling:
  Ren et al., "NeRF On-the-go: Exploiting Uncertainty for Distractor-free NeRFs in the Wild," CVPR 2024.
  Link: https://openaccess.thecvf.com/content/CVPR2024/html/Ren_NeRF_On-the-go_Exploiting_Uncertainty_for_Distractor-free_NeRFs_in_the_Wild_CVPR_2024_paper.html

## 15. Bottom Line

현재 COIN 구현은 다음처럼 이해하는 것이 가장 정확하다:

```text
task-feature adaptation under photometric shift
+ context-assisted feature repair under partial occlusion
+ sparse visibility supervision tied to pose labels
```

이 방향 자체는 이미 산업 적용 관점에서도 충분히 합리적이다.

현재 코드에서 가장 중요한 open question은 두 가지다.

1. visibility가 다시 symmetric reconstruction-driven gradient를 받아야 하는가
2. Phase 2에서 Phase 1의 visibility / reconstruction behavior를 명시적으로
   보존해야 하는가
