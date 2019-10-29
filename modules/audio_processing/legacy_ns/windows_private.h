/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_LEGACY_NS_WINDOWS_PRIVATE_H_
#define MODULES_AUDIO_PROCESSING_LEGACY_NS_WINDOWS_PRIVATE_H_

// Hanning window for 4ms 16kHz
static const float kHanning64w128[128] = {
    0.00000000000000f, 0.02454122852291f, 0.04906767432742f, 0.07356456359967f,
    0.09801714032956f, 0.12241067519922f, 0.14673047445536f, 0.17096188876030f,
    0.19509032201613f, 0.21910124015687f, 0.24298017990326f, 0.26671275747490f,
    0.29028467725446f, 0.31368174039889f, 0.33688985339222f, 0.35989503653499f,
    0.38268343236509f, 0.40524131400499f, 0.42755509343028f, 0.44961132965461f,
    0.47139673682600f, 0.49289819222978f, 0.51410274419322f, 0.53499761988710f,
    0.55557023301960f, 0.57580819141785f, 0.59569930449243f, 0.61523159058063f,
    0.63439328416365f, 0.65317284295378f, 0.67155895484702f, 0.68954054473707f,
    0.70710678118655f, 0.72424708295147f, 0.74095112535496f, 0.75720884650648f,
    0.77301045336274f, 0.78834642762661f, 0.80320753148064f, 0.81758481315158f,
    0.83146961230255f, 0.84485356524971f, 0.85772861000027f, 0.87008699110871f,
    0.88192126434835f, 0.89322430119552f, 0.90398929312344f, 0.91420975570353f,
    0.92387953251129f, 0.93299279883474f, 0.94154406518302f, 0.94952818059304f,
    0.95694033573221f, 0.96377606579544f, 0.97003125319454f, 0.97570213003853f,
    0.98078528040323f, 0.98527764238894f, 0.98917650996478f, 0.99247953459871f,
    0.99518472667220f, 0.99729045667869f, 0.99879545620517f, 0.99969881869620f,
    1.00000000000000f, 0.99969881869620f, 0.99879545620517f, 0.99729045667869f,
    0.99518472667220f, 0.99247953459871f, 0.98917650996478f, 0.98527764238894f,
    0.98078528040323f, 0.97570213003853f, 0.97003125319454f, 0.96377606579544f,
    0.95694033573221f, 0.94952818059304f, 0.94154406518302f, 0.93299279883474f,
    0.92387953251129f, 0.91420975570353f, 0.90398929312344f, 0.89322430119552f,
    0.88192126434835f, 0.87008699110871f, 0.85772861000027f, 0.84485356524971f,
    0.83146961230255f, 0.81758481315158f, 0.80320753148064f, 0.78834642762661f,
    0.77301045336274f, 0.75720884650648f, 0.74095112535496f, 0.72424708295147f,
    0.70710678118655f, 0.68954054473707f, 0.67155895484702f, 0.65317284295378f,
    0.63439328416365f, 0.61523159058063f, 0.59569930449243f, 0.57580819141785f,
    0.55557023301960f, 0.53499761988710f, 0.51410274419322f, 0.49289819222978f,
    0.47139673682600f, 0.44961132965461f, 0.42755509343028f, 0.40524131400499f,
    0.38268343236509f, 0.35989503653499f, 0.33688985339222f, 0.31368174039889f,
    0.29028467725446f, 0.26671275747490f, 0.24298017990326f, 0.21910124015687f,
    0.19509032201613f, 0.17096188876030f, 0.14673047445536f, 0.12241067519922f,
    0.09801714032956f, 0.07356456359967f, 0.04906767432742f, 0.02454122852291f};

// hybrib Hanning & flat window
static const float kBlocks80w128[128] = {
    0.00000000f, 0.03271908f, 0.06540313f, 0.09801714f, 0.13052619f,
    0.16289547f, 0.19509032f, 0.22707626f, 0.25881905f, 0.29028468f,
    0.32143947f, 0.35225005f, 0.38268343f, 0.41270703f, 0.44228869f,
    0.47139674f, 0.50000000f, 0.52806785f, 0.55557023f, 0.58247770f,
    0.60876143f, 0.63439328f, 0.65934582f, 0.68359230f, 0.70710678f,
    0.72986407f, 0.75183981f, 0.77301045f, 0.79335334f, 0.81284668f,
    0.83146961f, 0.84920218f, 0.86602540f, 0.88192126f, 0.89687274f,
    0.91086382f, 0.92387953f, 0.93590593f, 0.94693013f, 0.95694034f,
    0.96592583f, 0.97387698f, 0.98078528f, 0.98664333f, 0.99144486f,
    0.99518473f, 0.99785892f, 0.99946459f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 0.99946459f, 0.99785892f, 0.99518473f, 0.99144486f,
    0.98664333f, 0.98078528f, 0.97387698f, 0.96592583f, 0.95694034f,
    0.94693013f, 0.93590593f, 0.92387953f, 0.91086382f, 0.89687274f,
    0.88192126f, 0.86602540f, 0.84920218f, 0.83146961f, 0.81284668f,
    0.79335334f, 0.77301045f, 0.75183981f, 0.72986407f, 0.70710678f,
    0.68359230f, 0.65934582f, 0.63439328f, 0.60876143f, 0.58247770f,
    0.55557023f, 0.52806785f, 0.50000000f, 0.47139674f, 0.44228869f,
    0.41270703f, 0.38268343f, 0.35225005f, 0.32143947f, 0.29028468f,
    0.25881905f, 0.22707626f, 0.19509032f, 0.16289547f, 0.13052619f,
    0.09801714f, 0.06540313f, 0.03271908f};

// hybrib Hanning & flat window
static const float kBlocks160w256[256] = {
    0.00000000f, 0.01636173f, 0.03271908f, 0.04906767f, 0.06540313f,
    0.08172107f, 0.09801714f, 0.11428696f, 0.13052619f, 0.14673047f,
    0.16289547f, 0.17901686f, 0.19509032f, 0.21111155f, 0.22707626f,
    0.24298018f, 0.25881905f, 0.27458862f, 0.29028468f, 0.30590302f,
    0.32143947f, 0.33688985f, 0.35225005f, 0.36751594f, 0.38268343f,
    0.39774847f, 0.41270703f, 0.42755509f, 0.44228869f, 0.45690388f,
    0.47139674f, 0.48576339f, 0.50000000f, 0.51410274f, 0.52806785f,
    0.54189158f, 0.55557023f, 0.56910015f, 0.58247770f, 0.59569930f,
    0.60876143f, 0.62166057f, 0.63439328f, 0.64695615f, 0.65934582f,
    0.67155895f, 0.68359230f, 0.69544264f, 0.70710678f, 0.71858162f,
    0.72986407f, 0.74095113f, 0.75183981f, 0.76252720f, 0.77301045f,
    0.78328675f, 0.79335334f, 0.80320753f, 0.81284668f, 0.82226822f,
    0.83146961f, 0.84044840f, 0.84920218f, 0.85772861f, 0.86602540f,
    0.87409034f, 0.88192126f, 0.88951608f, 0.89687274f, 0.90398929f,
    0.91086382f, 0.91749450f, 0.92387953f, 0.93001722f, 0.93590593f,
    0.94154407f, 0.94693013f, 0.95206268f, 0.95694034f, 0.96156180f,
    0.96592583f, 0.97003125f, 0.97387698f, 0.97746197f, 0.98078528f,
    0.98384601f, 0.98664333f, 0.98917651f, 0.99144486f, 0.99344778f,
    0.99518473f, 0.99665524f, 0.99785892f, 0.99879546f, 0.99946459f,
    0.99986614f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 0.99986614f, 0.99946459f, 0.99879546f, 0.99785892f,
    0.99665524f, 0.99518473f, 0.99344778f, 0.99144486f, 0.98917651f,
    0.98664333f, 0.98384601f, 0.98078528f, 0.97746197f, 0.97387698f,
    0.97003125f, 0.96592583f, 0.96156180f, 0.95694034f, 0.95206268f,
    0.94693013f, 0.94154407f, 0.93590593f, 0.93001722f, 0.92387953f,
    0.91749450f, 0.91086382f, 0.90398929f, 0.89687274f, 0.88951608f,
    0.88192126f, 0.87409034f, 0.86602540f, 0.85772861f, 0.84920218f,
    0.84044840f, 0.83146961f, 0.82226822f, 0.81284668f, 0.80320753f,
    0.79335334f, 0.78328675f, 0.77301045f, 0.76252720f, 0.75183981f,
    0.74095113f, 0.72986407f, 0.71858162f, 0.70710678f, 0.69544264f,
    0.68359230f, 0.67155895f, 0.65934582f, 0.64695615f, 0.63439328f,
    0.62166057f, 0.60876143f, 0.59569930f, 0.58247770f, 0.56910015f,
    0.55557023f, 0.54189158f, 0.52806785f, 0.51410274f, 0.50000000f,
    0.48576339f, 0.47139674f, 0.45690388f, 0.44228869f, 0.42755509f,
    0.41270703f, 0.39774847f, 0.38268343f, 0.36751594f, 0.35225005f,
    0.33688985f, 0.32143947f, 0.30590302f, 0.29028468f, 0.27458862f,
    0.25881905f, 0.24298018f, 0.22707626f, 0.21111155f, 0.19509032f,
    0.17901686f, 0.16289547f, 0.14673047f, 0.13052619f, 0.11428696f,
    0.09801714f, 0.08172107f, 0.06540313f, 0.04906767f, 0.03271908f,
    0.01636173f};

// hybrib Hanning & flat window: for 20ms
static const float kBlocks320w512[512] = {
    0.00000000f, 0.00818114f, 0.01636173f, 0.02454123f, 0.03271908f,
    0.04089475f, 0.04906767f, 0.05723732f, 0.06540313f, 0.07356456f,
    0.08172107f, 0.08987211f, 0.09801714f, 0.10615561f, 0.11428696f,
    0.12241068f, 0.13052619f, 0.13863297f, 0.14673047f, 0.15481816f,
    0.16289547f, 0.17096189f, 0.17901686f, 0.18705985f, 0.19509032f,
    0.20310773f, 0.21111155f, 0.21910124f, 0.22707626f, 0.23503609f,
    0.24298018f, 0.25090801f, 0.25881905f, 0.26671276f, 0.27458862f,
    0.28244610f, 0.29028468f, 0.29810383f, 0.30590302f, 0.31368174f,
    0.32143947f, 0.32917568f, 0.33688985f, 0.34458148f, 0.35225005f,
    0.35989504f, 0.36751594f, 0.37511224f, 0.38268343f, 0.39022901f,
    0.39774847f, 0.40524131f, 0.41270703f, 0.42014512f, 0.42755509f,
    0.43493645f, 0.44228869f, 0.44961133f, 0.45690388f, 0.46416584f,
    0.47139674f, 0.47859608f, 0.48576339f, 0.49289819f, 0.50000000f,
    0.50706834f, 0.51410274f, 0.52110274f, 0.52806785f, 0.53499762f,
    0.54189158f, 0.54874927f, 0.55557023f, 0.56235401f, 0.56910015f,
    0.57580819f, 0.58247770f, 0.58910822f, 0.59569930f, 0.60225052f,
    0.60876143f, 0.61523159f, 0.62166057f, 0.62804795f, 0.63439328f,
    0.64069616f, 0.64695615f, 0.65317284f, 0.65934582f, 0.66547466f,
    0.67155895f, 0.67759830f, 0.68359230f, 0.68954054f, 0.69544264f,
    0.70129818f, 0.70710678f, 0.71286806f, 0.71858162f, 0.72424708f,
    0.72986407f, 0.73543221f, 0.74095113f, 0.74642045f, 0.75183981f,
    0.75720885f, 0.76252720f, 0.76779452f, 0.77301045f, 0.77817464f,
    0.78328675f, 0.78834643f, 0.79335334f, 0.79830715f, 0.80320753f,
    0.80805415f, 0.81284668f, 0.81758481f, 0.82226822f, 0.82689659f,
    0.83146961f, 0.83598698f, 0.84044840f, 0.84485357f, 0.84920218f,
    0.85349396f, 0.85772861f, 0.86190585f, 0.86602540f, 0.87008699f,
    0.87409034f, 0.87803519f, 0.88192126f, 0.88574831f, 0.88951608f,
    0.89322430f, 0.89687274f, 0.90046115f, 0.90398929f, 0.90745693f,
    0.91086382f, 0.91420976f, 0.91749450f, 0.92071783f, 0.92387953f,
    0.92697940f, 0.93001722f, 0.93299280f, 0.93590593f, 0.93875641f,
    0.94154407f, 0.94426870f, 0.94693013f, 0.94952818f, 0.95206268f,
    0.95453345f, 0.95694034f, 0.95928317f, 0.96156180f, 0.96377607f,
    0.96592583f, 0.96801094f, 0.97003125f, 0.97198664f, 0.97387698f,
    0.97570213f, 0.97746197f, 0.97915640f, 0.98078528f, 0.98234852f,
    0.98384601f, 0.98527764f, 0.98664333f, 0.98794298f, 0.98917651f,
    0.99034383f, 0.99144486f, 0.99247953f, 0.99344778f, 0.99434953f,
    0.99518473f, 0.99595331f, 0.99665524f, 0.99729046f, 0.99785892f,
    0.99836060f, 0.99879546f, 0.99916346f, 0.99946459f, 0.99969882f,
    0.99986614f, 0.99996653f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f, 1.00000000f,
    1.00000000f, 0.99996653f, 0.99986614f, 0.99969882f, 0.99946459f,
    0.99916346f, 0.99879546f, 0.99836060f, 0.99785892f, 0.99729046f,
    0.99665524f, 0.99595331f, 0.99518473f, 0.99434953f, 0.99344778f,
    0.99247953f, 0.99144486f, 0.99034383f, 0.98917651f, 0.98794298f,
    0.98664333f, 0.98527764f, 0.98384601f, 0.98234852f, 0.98078528f,
    0.97915640f, 0.97746197f, 0.97570213f, 0.97387698f, 0.97198664f,
    0.97003125f, 0.96801094f, 0.96592583f, 0.96377607f, 0.96156180f,
    0.95928317f, 0.95694034f, 0.95453345f, 0.95206268f, 0.94952818f,
    0.94693013f, 0.94426870f, 0.94154407f, 0.93875641f, 0.93590593f,
    0.93299280f, 0.93001722f, 0.92697940f, 0.92387953f, 0.92071783f,
    0.91749450f, 0.91420976f, 0.91086382f, 0.90745693f, 0.90398929f,
    0.90046115f, 0.89687274f, 0.89322430f, 0.88951608f, 0.88574831f,
    0.88192126f, 0.87803519f, 0.87409034f, 0.87008699f, 0.86602540f,
    0.86190585f, 0.85772861f, 0.85349396f, 0.84920218f, 0.84485357f,
    0.84044840f, 0.83598698f, 0.83146961f, 0.82689659f, 0.82226822f,
    0.81758481f, 0.81284668f, 0.80805415f, 0.80320753f, 0.79830715f,
    0.79335334f, 0.78834643f, 0.78328675f, 0.77817464f, 0.77301045f,
    0.76779452f, 0.76252720f, 0.75720885f, 0.75183981f, 0.74642045f,
    0.74095113f, 0.73543221f, 0.72986407f, 0.72424708f, 0.71858162f,
    0.71286806f, 0.70710678f, 0.70129818f, 0.69544264f, 0.68954054f,
    0.68359230f, 0.67759830f, 0.67155895f, 0.66547466f, 0.65934582f,
    0.65317284f, 0.64695615f, 0.64069616f, 0.63439328f, 0.62804795f,
    0.62166057f, 0.61523159f, 0.60876143f, 0.60225052f, 0.59569930f,
    0.58910822f, 0.58247770f, 0.57580819f, 0.56910015f, 0.56235401f,
    0.55557023f, 0.54874927f, 0.54189158f, 0.53499762f, 0.52806785f,
    0.52110274f, 0.51410274f, 0.50706834f, 0.50000000f, 0.49289819f,
    0.48576339f, 0.47859608f, 0.47139674f, 0.46416584f, 0.45690388f,
    0.44961133f, 0.44228869f, 0.43493645f, 0.42755509f, 0.42014512f,
    0.41270703f, 0.40524131f, 0.39774847f, 0.39022901f, 0.38268343f,
    0.37511224f, 0.36751594f, 0.35989504f, 0.35225005f, 0.34458148f,
    0.33688985f, 0.32917568f, 0.32143947f, 0.31368174f, 0.30590302f,
    0.29810383f, 0.29028468f, 0.28244610f, 0.27458862f, 0.26671276f,
    0.25881905f, 0.25090801f, 0.24298018f, 0.23503609f, 0.22707626f,
    0.21910124f, 0.21111155f, 0.20310773f, 0.19509032f, 0.18705985f,
    0.17901686f, 0.17096189f, 0.16289547f, 0.15481816f, 0.14673047f,
    0.13863297f, 0.13052619f, 0.12241068f, 0.11428696f, 0.10615561f,
    0.09801714f, 0.08987211f, 0.08172107f, 0.07356456f, 0.06540313f,
    0.05723732f, 0.04906767f, 0.04089475f, 0.03271908f, 0.02454123f,
    0.01636173f, 0.00818114f};

// Hanning window: for 15ms at 16kHz with symmetric zeros
static const float kBlocks240w512[512] = {
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00654494f, 0.01308960f, 0.01963369f,
    0.02617695f, 0.03271908f, 0.03925982f, 0.04579887f, 0.05233596f,
    0.05887080f, 0.06540313f, 0.07193266f, 0.07845910f, 0.08498218f,
    0.09150162f, 0.09801714f, 0.10452846f, 0.11103531f, 0.11753740f,
    0.12403446f, 0.13052620f, 0.13701233f, 0.14349262f, 0.14996676f,
    0.15643448f, 0.16289547f, 0.16934951f, 0.17579629f, 0.18223552f,
    0.18866697f, 0.19509032f, 0.20150533f, 0.20791170f, 0.21430916f,
    0.22069745f, 0.22707628f, 0.23344538f, 0.23980446f, 0.24615330f,
    0.25249159f, 0.25881904f, 0.26513544f, 0.27144045f, 0.27773386f,
    0.28401536f, 0.29028466f, 0.29654160f, 0.30278578f, 0.30901700f,
    0.31523499f, 0.32143945f, 0.32763019f, 0.33380687f, 0.33996925f,
    0.34611708f, 0.35225007f, 0.35836795f, 0.36447051f, 0.37055743f,
    0.37662852f, 0.38268346f, 0.38872197f, 0.39474389f, 0.40074885f,
    0.40673664f, 0.41270703f, 0.41865975f, 0.42459452f, 0.43051112f,
    0.43640924f, 0.44228873f, 0.44814920f, 0.45399052f, 0.45981237f,
    0.46561453f, 0.47139674f, 0.47715878f, 0.48290035f, 0.48862126f,
    0.49432120f, 0.50000000f, 0.50565743f, 0.51129311f, 0.51690692f,
    0.52249855f, 0.52806789f, 0.53361452f, 0.53913832f, 0.54463905f,
    0.55011642f, 0.55557024f, 0.56100029f, 0.56640625f, 0.57178795f,
    0.57714522f, 0.58247769f, 0.58778524f, 0.59306765f, 0.59832460f,
    0.60355598f, 0.60876143f, 0.61394083f, 0.61909395f, 0.62422055f,
    0.62932038f, 0.63439333f, 0.63943899f, 0.64445734f, 0.64944810f,
    0.65441096f, 0.65934587f, 0.66425246f, 0.66913062f, 0.67398012f,
    0.67880076f, 0.68359232f, 0.68835455f, 0.69308740f, 0.69779050f,
    0.70246369f, 0.70710677f, 0.71171963f, 0.71630198f, 0.72085363f,
    0.72537440f, 0.72986406f, 0.73432255f, 0.73874950f, 0.74314487f,
    0.74750835f, 0.75183982f, 0.75613910f, 0.76040596f, 0.76464027f,
    0.76884186f, 0.77301043f, 0.77714598f, 0.78124821f, 0.78531694f,
    0.78935206f, 0.79335338f, 0.79732066f, 0.80125386f, 0.80515265f,
    0.80901700f, 0.81284672f, 0.81664157f, 0.82040149f, 0.82412618f,
    0.82781565f, 0.83146966f, 0.83508795f, 0.83867061f, 0.84221727f,
    0.84572780f, 0.84920216f, 0.85264021f, 0.85604161f, 0.85940641f,
    0.86273444f, 0.86602545f, 0.86927933f, 0.87249607f, 0.87567532f,
    0.87881714f, 0.88192129f, 0.88498765f, 0.88801610f, 0.89100653f,
    0.89395881f, 0.89687276f, 0.89974827f, 0.90258533f, 0.90538365f,
    0.90814316f, 0.91086388f, 0.91354549f, 0.91618794f, 0.91879123f,
    0.92135513f, 0.92387950f, 0.92636442f, 0.92880958f, 0.93121493f,
    0.93358046f, 0.93590593f, 0.93819135f, 0.94043654f, 0.94264150f,
    0.94480604f, 0.94693011f, 0.94901365f, 0.95105654f, 0.95305866f,
    0.95501995f, 0.95694035f, 0.95881975f, 0.96065807f, 0.96245527f,
    0.96421117f, 0.96592581f, 0.96759909f, 0.96923089f, 0.97082120f,
    0.97236991f, 0.97387701f, 0.97534233f, 0.97676587f, 0.97814763f,
    0.97948742f, 0.98078531f, 0.98204112f, 0.98325491f, 0.98442656f,
    0.98555607f, 0.98664331f, 0.98768836f, 0.98869103f, 0.98965138f,
    0.99056935f, 0.99144489f, 0.99227792f, 0.99306846f, 0.99381649f,
    0.99452192f, 0.99518472f, 0.99580491f, 0.99638247f, 0.99691731f,
    0.99740952f, 0.99785894f, 0.99826562f, 0.99862951f, 0.99895066f,
    0.99922901f, 0.99946457f, 0.99965733f, 0.99980724f, 0.99991435f,
    0.99997860f, 1.00000000f, 0.99997860f, 0.99991435f, 0.99980724f,
    0.99965733f, 0.99946457f, 0.99922901f, 0.99895066f, 0.99862951f,
    0.99826562f, 0.99785894f, 0.99740946f, 0.99691731f, 0.99638247f,
    0.99580491f, 0.99518472f, 0.99452192f, 0.99381644f, 0.99306846f,
    0.99227792f, 0.99144489f, 0.99056935f, 0.98965138f, 0.98869103f,
    0.98768836f, 0.98664331f, 0.98555607f, 0.98442656f, 0.98325491f,
    0.98204112f, 0.98078525f, 0.97948742f, 0.97814757f, 0.97676587f,
    0.97534227f, 0.97387695f, 0.97236991f, 0.97082120f, 0.96923089f,
    0.96759909f, 0.96592581f, 0.96421117f, 0.96245521f, 0.96065807f,
    0.95881969f, 0.95694029f, 0.95501995f, 0.95305860f, 0.95105648f,
    0.94901365f, 0.94693011f, 0.94480604f, 0.94264150f, 0.94043654f,
    0.93819129f, 0.93590593f, 0.93358046f, 0.93121493f, 0.92880952f,
    0.92636436f, 0.92387950f, 0.92135507f, 0.91879123f, 0.91618794f,
    0.91354543f, 0.91086382f, 0.90814310f, 0.90538365f, 0.90258527f,
    0.89974827f, 0.89687276f, 0.89395875f, 0.89100647f, 0.88801610f,
    0.88498759f, 0.88192123f, 0.87881714f, 0.87567532f, 0.87249595f,
    0.86927933f, 0.86602539f, 0.86273432f, 0.85940641f, 0.85604161f,
    0.85264009f, 0.84920216f, 0.84572780f, 0.84221715f, 0.83867055f,
    0.83508795f, 0.83146954f, 0.82781565f, 0.82412612f, 0.82040137f,
    0.81664157f, 0.81284660f, 0.80901700f, 0.80515265f, 0.80125374f,
    0.79732066f, 0.79335332f, 0.78935200f, 0.78531694f, 0.78124815f,
    0.77714586f, 0.77301049f, 0.76884180f, 0.76464021f, 0.76040596f,
    0.75613904f, 0.75183970f, 0.74750835f, 0.74314481f, 0.73874938f,
    0.73432249f, 0.72986400f, 0.72537428f, 0.72085363f, 0.71630186f,
    0.71171951f, 0.70710677f, 0.70246363f, 0.69779032f, 0.69308734f,
    0.68835449f, 0.68359220f, 0.67880070f, 0.67398006f, 0.66913044f,
    0.66425240f, 0.65934575f, 0.65441096f, 0.64944804f, 0.64445722f,
    0.63943905f, 0.63439327f, 0.62932026f, 0.62422055f, 0.61909389f,
    0.61394072f, 0.60876143f, 0.60355592f, 0.59832448f, 0.59306765f,
    0.58778518f, 0.58247757f, 0.57714522f, 0.57178789f, 0.56640613f,
    0.56100023f, 0.55557019f, 0.55011630f, 0.54463905f, 0.53913826f,
    0.53361434f, 0.52806783f, 0.52249849f, 0.51690674f, 0.51129305f,
    0.50565726f, 0.50000006f, 0.49432117f, 0.48862115f, 0.48290038f,
    0.47715873f, 0.47139663f, 0.46561456f, 0.45981231f, 0.45399037f,
    0.44814920f, 0.44228864f, 0.43640912f, 0.43051112f, 0.42459446f,
    0.41865960f, 0.41270703f, 0.40673658f, 0.40074870f, 0.39474386f,
    0.38872188f, 0.38268328f, 0.37662849f, 0.37055734f, 0.36447033f,
    0.35836792f, 0.35224995f, 0.34611690f, 0.33996922f, 0.33380675f,
    0.32763001f, 0.32143945f, 0.31523487f, 0.30901679f, 0.30278572f,
    0.29654145f, 0.29028472f, 0.28401530f, 0.27773371f, 0.27144048f,
    0.26513538f, 0.25881892f, 0.25249159f, 0.24615324f, 0.23980433f,
    0.23344538f, 0.22707619f, 0.22069728f, 0.21430916f, 0.20791161f,
    0.20150517f, 0.19509031f, 0.18866688f, 0.18223536f, 0.17579627f,
    0.16934940f, 0.16289529f, 0.15643445f, 0.14996666f, 0.14349243f,
    0.13701232f, 0.13052608f, 0.12403426f, 0.11753736f, 0.11103519f,
    0.10452849f, 0.09801710f, 0.09150149f, 0.08498220f, 0.07845904f,
    0.07193252f, 0.06540315f, 0.05887074f, 0.05233581f, 0.04579888f,
    0.03925974f, 0.03271893f, 0.02617695f, 0.01963361f, 0.01308943f,
    0.00654493f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f};

// Hanning window: for 30ms with 1024 fft with symmetric zeros at 16kHz
static const float kBlocks480w1024[1024] = {
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00327249f, 0.00654494f,
    0.00981732f, 0.01308960f, 0.01636173f, 0.01963369f, 0.02290544f,
    0.02617695f, 0.02944817f, 0.03271908f, 0.03598964f, 0.03925982f,
    0.04252957f, 0.04579887f, 0.04906768f, 0.05233596f, 0.05560368f,
    0.05887080f, 0.06213730f, 0.06540313f, 0.06866825f, 0.07193266f,
    0.07519628f, 0.07845910f, 0.08172107f, 0.08498218f, 0.08824237f,
    0.09150162f, 0.09475989f, 0.09801714f, 0.10127335f, 0.10452846f,
    0.10778246f, 0.11103531f, 0.11428697f, 0.11753740f, 0.12078657f,
    0.12403446f, 0.12728101f, 0.13052620f, 0.13376999f, 0.13701233f,
    0.14025325f, 0.14349262f, 0.14673047f, 0.14996676f, 0.15320145f,
    0.15643448f, 0.15966582f, 0.16289547f, 0.16612339f, 0.16934951f,
    0.17257382f, 0.17579629f, 0.17901687f, 0.18223552f, 0.18545224f,
    0.18866697f, 0.19187967f, 0.19509032f, 0.19829889f, 0.20150533f,
    0.20470962f, 0.20791170f, 0.21111156f, 0.21430916f, 0.21750447f,
    0.22069745f, 0.22388805f, 0.22707628f, 0.23026206f, 0.23344538f,
    0.23662618f, 0.23980446f, 0.24298020f, 0.24615330f, 0.24932377f,
    0.25249159f, 0.25565669f, 0.25881904f, 0.26197866f, 0.26513544f,
    0.26828939f, 0.27144045f, 0.27458861f, 0.27773386f, 0.28087610f,
    0.28401536f, 0.28715158f, 0.29028466f, 0.29341471f, 0.29654160f,
    0.29966527f, 0.30278578f, 0.30590302f, 0.30901700f, 0.31212768f,
    0.31523499f, 0.31833893f, 0.32143945f, 0.32453656f, 0.32763019f,
    0.33072028f, 0.33380687f, 0.33688986f, 0.33996925f, 0.34304500f,
    0.34611708f, 0.34918544f, 0.35225007f, 0.35531089f, 0.35836795f,
    0.36142117f, 0.36447051f, 0.36751595f, 0.37055743f, 0.37359497f,
    0.37662852f, 0.37965801f, 0.38268346f, 0.38570479f, 0.38872197f,
    0.39173502f, 0.39474389f, 0.39774847f, 0.40074885f, 0.40374491f,
    0.40673664f, 0.40972406f, 0.41270703f, 0.41568562f, 0.41865975f,
    0.42162940f, 0.42459452f, 0.42755508f, 0.43051112f, 0.43346250f,
    0.43640924f, 0.43935132f, 0.44228873f, 0.44522133f, 0.44814920f,
    0.45107228f, 0.45399052f, 0.45690390f, 0.45981237f, 0.46271592f,
    0.46561453f, 0.46850815f, 0.47139674f, 0.47428030f, 0.47715878f,
    0.48003215f, 0.48290035f, 0.48576337f, 0.48862126f, 0.49147385f,
    0.49432120f, 0.49716330f, 0.50000000f, 0.50283140f, 0.50565743f,
    0.50847799f, 0.51129311f, 0.51410276f, 0.51690692f, 0.51970553f,
    0.52249855f, 0.52528602f, 0.52806789f, 0.53084403f, 0.53361452f,
    0.53637928f, 0.53913832f, 0.54189163f, 0.54463905f, 0.54738063f,
    0.55011642f, 0.55284631f, 0.55557024f, 0.55828828f, 0.56100029f,
    0.56370628f, 0.56640625f, 0.56910014f, 0.57178795f, 0.57446963f,
    0.57714522f, 0.57981455f, 0.58247769f, 0.58513463f, 0.58778524f,
    0.59042960f, 0.59306765f, 0.59569931f, 0.59832460f, 0.60094351f,
    0.60355598f, 0.60616195f, 0.60876143f, 0.61135441f, 0.61394083f,
    0.61652070f, 0.61909395f, 0.62166059f, 0.62422055f, 0.62677383f,
    0.62932038f, 0.63186020f, 0.63439333f, 0.63691956f, 0.63943899f,
    0.64195162f, 0.64445734f, 0.64695615f, 0.64944810f, 0.65193301f,
    0.65441096f, 0.65688187f, 0.65934587f, 0.66180271f, 0.66425246f,
    0.66669512f, 0.66913062f, 0.67155898f, 0.67398012f, 0.67639405f,
    0.67880076f, 0.68120021f, 0.68359232f, 0.68597710f, 0.68835455f,
    0.69072467f, 0.69308740f, 0.69544262f, 0.69779050f, 0.70013082f,
    0.70246369f, 0.70478904f, 0.70710677f, 0.70941699f, 0.71171963f,
    0.71401459f, 0.71630198f, 0.71858168f, 0.72085363f, 0.72311789f,
    0.72537440f, 0.72762316f, 0.72986406f, 0.73209721f, 0.73432255f,
    0.73653996f, 0.73874950f, 0.74095118f, 0.74314487f, 0.74533057f,
    0.74750835f, 0.74967808f, 0.75183982f, 0.75399351f, 0.75613910f,
    0.75827658f, 0.76040596f, 0.76252723f, 0.76464027f, 0.76674515f,
    0.76884186f, 0.77093029f, 0.77301043f, 0.77508241f, 0.77714598f,
    0.77920127f, 0.78124821f, 0.78328675f, 0.78531694f, 0.78733873f,
    0.78935206f, 0.79135692f, 0.79335338f, 0.79534125f, 0.79732066f,
    0.79929149f, 0.80125386f, 0.80320752f, 0.80515265f, 0.80708915f,
    0.80901700f, 0.81093621f, 0.81284672f, 0.81474853f, 0.81664157f,
    0.81852591f, 0.82040149f, 0.82226825f, 0.82412618f, 0.82597536f,
    0.82781565f, 0.82964706f, 0.83146966f, 0.83328325f, 0.83508795f,
    0.83688378f, 0.83867061f, 0.84044838f, 0.84221727f, 0.84397703f,
    0.84572780f, 0.84746957f, 0.84920216f, 0.85092574f, 0.85264021f,
    0.85434544f, 0.85604161f, 0.85772866f, 0.85940641f, 0.86107504f,
    0.86273444f, 0.86438453f, 0.86602545f, 0.86765707f, 0.86927933f,
    0.87089235f, 0.87249607f, 0.87409031f, 0.87567532f, 0.87725097f,
    0.87881714f, 0.88037390f, 0.88192129f, 0.88345921f, 0.88498765f,
    0.88650668f, 0.88801610f, 0.88951612f, 0.89100653f, 0.89248741f,
    0.89395881f, 0.89542055f, 0.89687276f, 0.89831537f, 0.89974827f,
    0.90117162f, 0.90258533f, 0.90398932f, 0.90538365f, 0.90676826f,
    0.90814316f, 0.90950841f, 0.91086388f, 0.91220951f, 0.91354549f,
    0.91487163f, 0.91618794f, 0.91749454f, 0.91879123f, 0.92007810f,
    0.92135513f, 0.92262226f, 0.92387950f, 0.92512691f, 0.92636442f,
    0.92759192f, 0.92880958f, 0.93001723f, 0.93121493f, 0.93240267f,
    0.93358046f, 0.93474817f, 0.93590593f, 0.93705362f, 0.93819135f,
    0.93931901f, 0.94043654f, 0.94154406f, 0.94264150f, 0.94372880f,
    0.94480604f, 0.94587320f, 0.94693011f, 0.94797695f, 0.94901365f,
    0.95004016f, 0.95105654f, 0.95206273f, 0.95305866f, 0.95404440f,
    0.95501995f, 0.95598525f, 0.95694035f, 0.95788521f, 0.95881975f,
    0.95974404f, 0.96065807f, 0.96156180f, 0.96245527f, 0.96333838f,
    0.96421117f, 0.96507370f, 0.96592581f, 0.96676767f, 0.96759909f,
    0.96842021f, 0.96923089f, 0.97003126f, 0.97082120f, 0.97160077f,
    0.97236991f, 0.97312868f, 0.97387701f, 0.97461486f, 0.97534233f,
    0.97605932f, 0.97676587f, 0.97746199f, 0.97814763f, 0.97882277f,
    0.97948742f, 0.98014158f, 0.98078531f, 0.98141843f, 0.98204112f,
    0.98265332f, 0.98325491f, 0.98384601f, 0.98442656f, 0.98499662f,
    0.98555607f, 0.98610497f, 0.98664331f, 0.98717111f, 0.98768836f,
    0.98819500f, 0.98869103f, 0.98917651f, 0.98965138f, 0.99011570f,
    0.99056935f, 0.99101239f, 0.99144489f, 0.99186671f, 0.99227792f,
    0.99267852f, 0.99306846f, 0.99344778f, 0.99381649f, 0.99417448f,
    0.99452192f, 0.99485862f, 0.99518472f, 0.99550015f, 0.99580491f,
    0.99609905f, 0.99638247f, 0.99665523f, 0.99691731f, 0.99716878f,
    0.99740952f, 0.99763954f, 0.99785894f, 0.99806762f, 0.99826562f,
    0.99845290f, 0.99862951f, 0.99879545f, 0.99895066f, 0.99909520f,
    0.99922901f, 0.99935216f, 0.99946457f, 0.99956632f, 0.99965733f,
    0.99973762f, 0.99980724f, 0.99986613f, 0.99991435f, 0.99995178f,
    0.99997860f, 0.99999464f, 1.00000000f, 0.99999464f, 0.99997860f,
    0.99995178f, 0.99991435f, 0.99986613f, 0.99980724f, 0.99973762f,
    0.99965733f, 0.99956632f, 0.99946457f, 0.99935216f, 0.99922901f,
    0.99909520f, 0.99895066f, 0.99879545f, 0.99862951f, 0.99845290f,
    0.99826562f, 0.99806762f, 0.99785894f, 0.99763954f, 0.99740946f,
    0.99716872f, 0.99691731f, 0.99665523f, 0.99638247f, 0.99609905f,
    0.99580491f, 0.99550015f, 0.99518472f, 0.99485862f, 0.99452192f,
    0.99417448f, 0.99381644f, 0.99344778f, 0.99306846f, 0.99267852f,
    0.99227792f, 0.99186671f, 0.99144489f, 0.99101239f, 0.99056935f,
    0.99011564f, 0.98965138f, 0.98917651f, 0.98869103f, 0.98819494f,
    0.98768836f, 0.98717111f, 0.98664331f, 0.98610497f, 0.98555607f,
    0.98499656f, 0.98442656f, 0.98384601f, 0.98325491f, 0.98265326f,
    0.98204112f, 0.98141843f, 0.98078525f, 0.98014158f, 0.97948742f,
    0.97882277f, 0.97814757f, 0.97746193f, 0.97676587f, 0.97605932f,
    0.97534227f, 0.97461486f, 0.97387695f, 0.97312862f, 0.97236991f,
    0.97160077f, 0.97082120f, 0.97003126f, 0.96923089f, 0.96842015f,
    0.96759909f, 0.96676761f, 0.96592581f, 0.96507365f, 0.96421117f,
    0.96333838f, 0.96245521f, 0.96156180f, 0.96065807f, 0.95974404f,
    0.95881969f, 0.95788515f, 0.95694029f, 0.95598525f, 0.95501995f,
    0.95404440f, 0.95305860f, 0.95206267f, 0.95105648f, 0.95004016f,
    0.94901365f, 0.94797695f, 0.94693011f, 0.94587314f, 0.94480604f,
    0.94372880f, 0.94264150f, 0.94154406f, 0.94043654f, 0.93931895f,
    0.93819129f, 0.93705362f, 0.93590593f, 0.93474817f, 0.93358046f,
    0.93240267f, 0.93121493f, 0.93001723f, 0.92880952f, 0.92759192f,
    0.92636436f, 0.92512691f, 0.92387950f, 0.92262226f, 0.92135507f,
    0.92007804f, 0.91879123f, 0.91749448f, 0.91618794f, 0.91487157f,
    0.91354543f, 0.91220951f, 0.91086382f, 0.90950835f, 0.90814310f,
    0.90676820f, 0.90538365f, 0.90398932f, 0.90258527f, 0.90117157f,
    0.89974827f, 0.89831525f, 0.89687276f, 0.89542055f, 0.89395875f,
    0.89248741f, 0.89100647f, 0.88951600f, 0.88801610f, 0.88650662f,
    0.88498759f, 0.88345915f, 0.88192123f, 0.88037384f, 0.87881714f,
    0.87725091f, 0.87567532f, 0.87409031f, 0.87249595f, 0.87089223f,
    0.86927933f, 0.86765701f, 0.86602539f, 0.86438447f, 0.86273432f,
    0.86107504f, 0.85940641f, 0.85772860f, 0.85604161f, 0.85434544f,
    0.85264009f, 0.85092574f, 0.84920216f, 0.84746951f, 0.84572780f,
    0.84397697f, 0.84221715f, 0.84044844f, 0.83867055f, 0.83688372f,
    0.83508795f, 0.83328319f, 0.83146954f, 0.82964706f, 0.82781565f,
    0.82597530f, 0.82412612f, 0.82226813f, 0.82040137f, 0.81852591f,
    0.81664157f, 0.81474847f, 0.81284660f, 0.81093609f, 0.80901700f,
    0.80708915f, 0.80515265f, 0.80320752f, 0.80125374f, 0.79929143f,
    0.79732066f, 0.79534125f, 0.79335332f, 0.79135686f, 0.78935200f,
    0.78733861f, 0.78531694f, 0.78328675f, 0.78124815f, 0.77920121f,
    0.77714586f, 0.77508223f, 0.77301049f, 0.77093029f, 0.76884180f,
    0.76674509f, 0.76464021f, 0.76252711f, 0.76040596f, 0.75827658f,
    0.75613904f, 0.75399339f, 0.75183970f, 0.74967796f, 0.74750835f,
    0.74533057f, 0.74314481f, 0.74095106f, 0.73874938f, 0.73653996f,
    0.73432249f, 0.73209721f, 0.72986400f, 0.72762305f, 0.72537428f,
    0.72311789f, 0.72085363f, 0.71858162f, 0.71630186f, 0.71401453f,
    0.71171951f, 0.70941705f, 0.70710677f, 0.70478898f, 0.70246363f,
    0.70013070f, 0.69779032f, 0.69544268f, 0.69308734f, 0.69072461f,
    0.68835449f, 0.68597704f, 0.68359220f, 0.68120021f, 0.67880070f,
    0.67639399f, 0.67398006f, 0.67155886f, 0.66913044f, 0.66669512f,
    0.66425240f, 0.66180259f, 0.65934575f, 0.65688181f, 0.65441096f,
    0.65193301f, 0.64944804f, 0.64695609f, 0.64445722f, 0.64195150f,
    0.63943905f, 0.63691956f, 0.63439327f, 0.63186014f, 0.62932026f,
    0.62677372f, 0.62422055f, 0.62166059f, 0.61909389f, 0.61652064f,
    0.61394072f, 0.61135429f, 0.60876143f, 0.60616189f, 0.60355592f,
    0.60094339f, 0.59832448f, 0.59569913f, 0.59306765f, 0.59042960f,
    0.58778518f, 0.58513451f, 0.58247757f, 0.57981461f, 0.57714522f,
    0.57446963f, 0.57178789f, 0.56910002f, 0.56640613f, 0.56370628f,
    0.56100023f, 0.55828822f, 0.55557019f, 0.55284619f, 0.55011630f,
    0.54738069f, 0.54463905f, 0.54189152f, 0.53913826f, 0.53637916f,
    0.53361434f, 0.53084403f, 0.52806783f, 0.52528596f, 0.52249849f,
    0.51970541f, 0.51690674f, 0.51410276f, 0.51129305f, 0.50847787f,
    0.50565726f, 0.50283122f, 0.50000006f, 0.49716327f, 0.49432117f,
    0.49147379f, 0.48862115f, 0.48576325f, 0.48290038f, 0.48003212f,
    0.47715873f, 0.47428021f, 0.47139663f, 0.46850798f, 0.46561456f,
    0.46271589f, 0.45981231f, 0.45690379f, 0.45399037f, 0.45107210f,
    0.44814920f, 0.44522130f, 0.44228864f, 0.43935123f, 0.43640912f,
    0.43346232f, 0.43051112f, 0.42755505f, 0.42459446f, 0.42162928f,
    0.41865960f, 0.41568545f, 0.41270703f, 0.40972400f, 0.40673658f,
    0.40374479f, 0.40074870f, 0.39774850f, 0.39474386f, 0.39173496f,
    0.38872188f, 0.38570464f, 0.38268328f, 0.37965804f, 0.37662849f,
    0.37359491f, 0.37055734f, 0.36751580f, 0.36447033f, 0.36142117f,
    0.35836792f, 0.35531086f, 0.35224995f, 0.34918529f, 0.34611690f,
    0.34304500f, 0.33996922f, 0.33688980f, 0.33380675f, 0.33072016f,
    0.32763001f, 0.32453656f, 0.32143945f, 0.31833887f, 0.31523487f,
    0.31212750f, 0.30901679f, 0.30590302f, 0.30278572f, 0.29966521f,
    0.29654145f, 0.29341453f, 0.29028472f, 0.28715155f, 0.28401530f,
    0.28087601f, 0.27773371f, 0.27458847f, 0.27144048f, 0.26828936f,
    0.26513538f, 0.26197854f, 0.25881892f, 0.25565651f, 0.25249159f,
    0.24932374f, 0.24615324f, 0.24298008f, 0.23980433f, 0.23662600f,
    0.23344538f, 0.23026201f, 0.22707619f, 0.22388794f, 0.22069728f,
    0.21750426f, 0.21430916f, 0.21111152f, 0.20791161f, 0.20470949f,
    0.20150517f, 0.19829892f, 0.19509031f, 0.19187963f, 0.18866688f,
    0.18545210f, 0.18223536f, 0.17901689f, 0.17579627f, 0.17257376f,
    0.16934940f, 0.16612324f, 0.16289529f, 0.15966584f, 0.15643445f,
    0.15320137f, 0.14996666f, 0.14673033f, 0.14349243f, 0.14025325f,
    0.13701232f, 0.13376991f, 0.13052608f, 0.12728085f, 0.12403426f,
    0.12078657f, 0.11753736f, 0.11428688f, 0.11103519f, 0.10778230f,
    0.10452849f, 0.10127334f, 0.09801710f, 0.09475980f, 0.09150149f,
    0.08824220f, 0.08498220f, 0.08172106f, 0.07845904f, 0.07519618f,
    0.07193252f, 0.06866808f, 0.06540315f, 0.06213728f, 0.05887074f,
    0.05560357f, 0.05233581f, 0.04906749f, 0.04579888f, 0.04252954f,
    0.03925974f, 0.03598953f, 0.03271893f, 0.02944798f, 0.02617695f,
    0.02290541f, 0.01963361f, 0.01636161f, 0.01308943f, 0.00981712f,
    0.00654493f, 0.00327244f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 0.00000000f, 0.00000000f};

#endif  // MODULES_AUDIO_PROCESSING_LEGACY_NS_WINDOWS_PRIVATE_H_
