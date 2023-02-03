% EHB 481 - Temel Haberleşme Sistemleri Tasarımı ve Uygulamaları (CRN :12070)
% Grup A1 :  İki-atlamalı (dual-hop) işbirlikli iletimin BPSK kullanarak SDR ile gerçeklenmesi
% Destination Node - Receiver 
%% SDR Receiver objesi - 1200 MHz'de işaret alma. Kaynak node'dan direk gelen işaret
clear

rx = comm.SDRuReceiver( ...
    'Platform','B200' ,...
    'SerialNum', '31F5C24', ...
    'CenterFrequency',1200e6,'SamplesPerFrame',25000, 'MasterClockRate', 56e6 ,...
    'Gain',70);

%% SamplesPerFrame kadar bilgi işareti alma işlemi

X = rx();
rxSig = double(X); % Alınan işaret ile aşağıdaki algoritmaların çalışabilmesi için 
% double data tipine çevrildi.

%% Down conversion işlemi  
t = (0:1:length(rxSig)-1)';
fromHighFreq = transpose(rxSig .* cos(2*pi*0.9*t));

%% Raised cosine ile filtre eşleme bloğu

% rcos parameters
beta = 0.55;
span = 10;
sps = 8;
g_R = rcosdesign(beta,span,sps,"sqrt");

% Göndericide rcos kullanıldığı için Rcos filtresi ile eşleme yapıldı.
afterRaisedCosine = conv( fromHighFreq , g_R );

%% Coarse Frequency Synchronization bloğu

fs = 56e6; % Symbol rate

psd = fftshift(abs(fft(afterRaisedCosine.^2,2^22)));
f = linspace(-fs/2.0, fs/2.0, length(psd));
[maxfreq,max_freq_ind] = max(psd);
max_freq = f(max_freq_ind);
Ts = 1/fs; % calc sample period
t = 0: Ts : (Ts*(length(afterRaisedCosine)-1)) ;% create time vector
frequencyCorrected = afterRaisedCosine .* exp(-1i*2*pi*max_freq*t/2.0);

%% Zaman senkranizasyonu
timingRecSignal = THAL_meyr_oeder_symbol_sync(8,128,frequencyCorrected);

%% Frame senkranizasyonu

% Zadoff Chu sequence
frame_header_length = 63
zadoff_chu = exp(-1i*pi*(0:frame_header_length-1).*((1:frame_header_length))/frame_header_length);

% Gelen işaretin zadoff chu kodları ile korelasyonunu alarak framelerin başlangıç noktaları bulundu. 
corr = xcorr(zadoff_chu,timingRecSignal(end:-1:1));
corr = corr / max(corr); % Korelasyon sonucu normalize edildi
corr_index = find(corr > 0.5); % Korelasyonu yüksek olan noktalar bulundu.
frame_start = corr_index(1)+1;

%% Faz kaymasının düzeltilmesi

% Pilot işareti
pltr = [1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1];

% Alınan pilot işaret ile gönderilen pilot işareti arasındaki fark
% kullanılarak alınan işaretteki faz kayması düzeltildi.
received_pilot = timingRecSignal(frame_start:frame_start+33);

% Faz farkının hesaplanması ve düzeltilmesi
direct_phase_difference = angle(sum(received_pilot*pltr'/sqrt(mean(abs(received_pilot).^2))));
direct_phase_corrected = timingRecSignal(frame_start+34:frame_start+1033) *exp(-1i*direct_phase_difference);

release(rx)

%% SDR Receiver objesi - 400 MHz'de işaret alma. Relay node'dan gelen işaret

rx = comm.SDRuReceiver( ...
    'Platform','B200' ,...
    'SerialNum','31F5C24', ...
    'CenterFrequency',400e6,'SamplesPerFrame',25000, 'MasterClockRate', 56e6 ,...
    'Gain',70);

%% SamplesPerFrame kadar bilgi işareti alma işlemi
X = rx();
rxSig = double(X); % Alınan işaret ile aşağıdaki algoritmaların çalışabilmesi için 
% double data tipine çevrildi.

%% Down conversion işlemi  
t = (0:1:length(rxSig)-1)';
fromHighFreq = transpose(rxSig .* cos(2*pi*0.9*t));

%% Raised cosine ile filtre eşleme bloğu

% rcos parameters
beta = 0.55;
span = 10;
sps = 8;
g_R = rcosdesign(beta,span,sps,"sqrt");

% Göndericide rcos kullanıldığı için Rcos filtresi ile eşleme yapıldı.
afterRaisedCosine = conv( fromHighFreq , g_R );

%% Coarse Frequency Synchronization bloğu

fs = 56e6;
psd = fftshift(abs(fft(afterRaisedCosine.^2,2^22)));
f = linspace(-fs/2.0, fs/2.0, length(psd));
[maxfreq,max_freq_ind] = max(psd);
max_freq = f(max_freq_ind);
Ts = 1/fs; % calc sample period
t = 0: Ts : (Ts*(length(afterRaisedCosine)-1)) ;% create time vector
frequencyCorrected = afterRaisedCosine .* exp(-1i*2*pi*max_freq*t/2.0);

%% Zaman senkranizasyonu
timingRecSignal = THAL_meyr_oeder_symbol_sync(8,128,frequencyCorrected);

%% Frame senkranizasyonu
% Zadoff Chu sequence
frame_header_length = 63;
zadoff_chu = exp(-1i*pi*(0:frame_header_length-1).*((1:frame_header_length))/frame_header_length);

pltr = [1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1];

corr = xcorr(zadoff_chu,timingRecSignal(end:-1:1));
corr = corr / max(corr);
corr_index = find(corr > 0.5);
frame_start = corr_index(1)+1;

%% Faz kaymasının düzeltilmesi
received_pilot = timingRecSignal(frame_start:frame_start+33);
middle_phase_difference = angle(sum(received_pilot*pltr'/sqrt(mean(abs(received_pilot).^2))));
middle_phase_corrected = timingRecSignal(frame_start+34:frame_start+1033) *exp(-1i*middle_phase_difference);

release(rx)

%% Alınan iki farklı işaretin ortak kullanılması

% Fazları düzeltilmiş iki işaret toplanarak bitler decode edildi 
% Equal gain combining
combined_phase_corrected = middle_phase_corrected + direct_phase_corrected;

combined_decoded_bits = sign(combined_phase_corrected);
received_bits = real(combined_decoded_bits) < 0;

%% Bit error rate'in hesaplanması
% Bit error rate
M = 2;
m = ceil(log2(M));
N = 1000;

seed = 10;
rng(seed);
send_data = randi([0 1],m,N);

[number,ratio] = biterr(send_data,received_bits)