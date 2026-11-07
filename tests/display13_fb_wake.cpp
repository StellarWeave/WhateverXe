#include "kern_display13_fb_irq.hpp"
#include <assert.h>
#include <stdio.h>
#include <map>
#include <vector>
#include <algorithm>
using namespace Display13FBIRQ;
struct IO {
    std::map<uint32_t,uint32_t> r;
    std::vector<uint32_t> reads,writes;
    uint32_t wells {0xFC0F};
    bool rejectEnable {};
    uint32_t rejectRegister {UINT32_MAX};
    IO() {
        r[Master]=r[GfxMaster]=0;r[DDIControl]=0x3300;r[0x64100]=r[0x64000]=0;
        r[TCControl]=r[NorthControl]=0;
        r[0xC4004]=~0U;r[0xC4008]=r[0xC400C]=0;
        for(unsigned p=0;p<3;++p) {
            r[0x70008+p*0x1000]=p ? 0 : 0xC0000000;
            r[PipeIIR(p)]=r[PipeIIR(p)+4]=0;r[PipeIIR(p)-4]=~0U;
            r[0x60400+p*0x1000]=r[0x70080+p*0x1000]=0;
            for(unsigned n=0;n<5;++n)r[0x70180+p*0x1000+n*0x100]=0;
            for(unsigned dsb=0;dsb<3;++dsb) {r[DSBInterrupt(p,dsb)]=0;r[DSBInterrupt(p,dsb)-0x20]=0;}
        }
        for(auto iir:{0x44448U,0x44468U,0x44478U,0x44488U}) {r[iir]=r[iir+4]=0;r[iir-4]=~0U;}
    }
    bool readable(uint32_t reg) {return reg<0x1000000;}
    void check(uint32_t reg) {
        for(unsigned p=0;p<3;++p)if((reg>=PipeIIR(p)-8 && reg<=PipeIIR(p)+4) ||
            (reg>=0x70000+p*0x1000 && reg<0x71000+p*0x1000))assert(powered(wells,p));
    }
    uint32_t read(uint32_t reg) {
        reads.push_back(reg);if(reg==Driver)return wells;check(reg);assert(r.count(reg));return r[reg];
    }
    void write(uint32_t reg,uint32_t value) {
        writes.push_back(reg);check(reg);assert(r.count(reg));
        if(reg==rejectRegister)return;
        if(reg==Master || reg==GfxMaster) {r[reg]=(r[reg]&~Enable)|(rejectEnable ? 0 : value&Enable);return;}
        if(reg==TCControl || reg==NorthControl) {r[reg]=(value&~0x333333U)|((r[reg]&0x333333U)&~value);return;}
        if(reg==DDIControl) {r[reg]=(value&~0x3333U)|((r[reg]&0x3333U)&~value);return;}
        for(unsigned p=0;p<3;++p) {
            if(reg==PipeIIR(p)) {r[reg]&=~value;return;}
            for(unsigned dsb=0;dsb<3;++dsb)if(reg==DSBInterrupt(p,dsb)) {
                r[reg]=(value&~0xFU)|((r[reg]&0xFU)&~value);return;
            }
        }
        if(reg==0xC4008 || reg==HPDBank)r[reg]&=~value;else r[reg]=value;
    }
};

// Local patch test (Slim 7 14IAP7): single-output display wake. The native driver enables interrupts
// before enablePipe sets PIPECONF; mirror of the wrapDisplay13FBEnableIRQ single-output branch.
static Status wakeEnable(IO &io,State &s) {
    if(s.begun && !s.faulted && !(s.enabled && s.active) && !Display13Transcoder::running(io,0,s.transcoders))
        return s.enabled && !s.active && (io.read(Master)&Enable) ? Status::Complete : hotplugOnly(io,s,false);
    return enable(io,s,0,0,1,1,0);
}
int main() {
    for(unsigned variant=0;variant<2;++variant) {
        IO io;State s;
        assert(enable(io,s,0,0,1,1,0)==Status::Complete && s.enabled && s.active==1);      // boot: GOP pipe A runs
        io.r[0x70008]=0;                                                                   // display off: pipe A stopped
        if(variant==0)assert(disable(io,s)==Status::Complete && hotplugOnly(io,s,false)==Status::Complete && s.enabled && !s.active);
        else assert(disable(io,s)==Status::Complete && !s.enabled);
        State old=s;IO oldIO=io;                                                           // upstream behaviour on wake
        assert(enable(oldIO,old,0,0,1,1,0)==Status::Topology && old.faulted && !(oldIO.r[Master]&Enable));
        assert(wakeEnable(io,s)==Status::Complete && !s.faulted && s.enabled && !s.active && (io.r[Master]&Enable));
        assert(wakeEnable(io,s)==Status::Complete && !s.faulted);                          // repeated native enable is idempotent
        io.r[0x70008]=0xC0000000;                                                          // enablePipe: PIPECONF running
        assert(enable(io,s,0,0,1,1,0)==Status::Complete && s.active==1 && !s.faulted);     // VBL-callback re-arm
        assert(vblank(io,s,0,true)==Status::Complete && (io.r[PipeIIR(0)+4]&1));
        assert(wakeEnable(io,s)==Status::Complete && s.active==1);                         // steady state unchanged
    }
    puts("PASS single-output display wake: enable before PIPECONF stays hotplug-only, upstream faults, VBL re-arm works");
}
