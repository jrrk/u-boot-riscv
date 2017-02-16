/**
 * @file
 * @copyright  Copyright 2016 GNSS Sensor Ltd. All right reserved.
 * @author     Sergey Khabarov - sergeykhbr@gmail.com
 * @brief      elf-file loader class implementation.
 */

#include "elfloader.h"
#include <iostream>

namespace debugger {

/** Class registration in the Core */
REGISTER_CLASS(ElfLoaderService)

ElfLoaderService::ElfLoaderService(const char *name) : IService(name) {
    registerInterface(static_cast<IElfLoader *>(this));
    registerAttribute("Tap", &tap_);
    registerAttribute("VerifyEna", &verify_ena_);
    tap_.make_string("");
    verify_ena_.make_boolean(false);
    itap_ = 0;
    sectionNames_ = NULL;
}

ElfLoaderService::~ElfLoaderService() {
}

void ElfLoaderService::postinitService() {
    IService *iserv = 
        static_cast<IService *>(RISCV_get_service(tap_.to_string()));
    if (!iserv) {
        RISCV_error("TAP service '%'s not found", tap_.to_string());
    }
    itap_ = static_cast<ITap *>(iserv->getInterface(IFACE_TAP));
    if (!itap_) {
        RISCV_error("ITap interface '%s' not found", tap_.to_string());
    }
}

int ElfLoaderService::loadFile(const char *filename) {
  int entry;
  bfd *tmp_bfd;                        //handler for libbfd
  tmp_bfd = bfd_openr (filename, NULL);
  if (tmp_bfd) {
    //check if the file is in format
    if (!bfd_check_format (tmp_bfd, bfd_object)) {
                if (bfd_get_error () != bfd_error_file_ambiguously_recognized) {
                        printf("Incompatible format\n");
                        exit(-1);
                }
    }
    printf ("exec file \"main.out\"'s format is %s\n",
        tmp_bfd->xvec->name);
    entry = bfd_get_start_address(tmp_bfd);
    printf("Entry point is at address %.8X\n", entry);

    /** Direct loading via tap interface: */
    int bytes_loaded = loadSections(tmp_bfd);
    RISCV_info("Loaded: %d B", bytes_loaded);

    bfd_close(tmp_bfd);
  }
  else
    perror(filename);
    return entry;
}


int ElfLoaderService::loadSections(bfd *tmp_bfd) {
    SectionHeaderType *sh;
    uint64_t total_bytes = 0;
    asection *s;
 
    /* load the corresponding section to memory */
    for (s = tmp_bfd->sections; s; s = s->next) {
      int flags = bfd_get_section_flags (tmp_bfd, s);
      int siz = (unsigned int) bfd_section_size (tmp_bfd, s);
      bfd_vma lma = (unsigned int) bfd_section_vma (tmp_bfd, s);
      bfd_vma vma = (unsigned int) bfd_section_vma (tmp_bfd, s);
      const char *nam = bfd_section_name (tmp_bfd, s);
      
        if (flags & (SEC_LOAD)) {
	  if (lma != vma) {
                printf ("loadable section %s: lma = 0x%08x (vma = 0x%08x)  size = 0x%08x\n",
			nam,
			(unsigned int) lma,
			(unsigned int) vma,
			siz);
            } else {
	    bfd_byte contents[siz];
	      
                printf ("loadable section %s: addr = 0x%08x  size = 0x%08x\n",
			nam,
			(unsigned int) vma,
			siz);
		if (bfd_get_section_contents (tmp_bfd, s,
				      contents, (file_ptr) 0,
				      siz))
		  {
		    if (strcmp(nam,".sdata.CSWTCH.80") && strcmp(nam, ".sdata.Stat")) // Yikes
			total_bytes += loadMemory(vma, contents, siz);
		  }

            }
        }
        else {
                 printf ("non-loadable section %s: addr = 0x%08x  size = 0x%08x\n",
			 nam,
			 (unsigned int) vma,
			 siz);
		 total_bytes += initMemory(vma, siz);
        }
    }                   //end of for loop
    return static_cast<int>(total_bytes);
}

uint64_t ElfLoaderService::loadMemory(uint64_t addr, 
                                      uint8_t *buf, uint64_t bufsz) {
    itap_->write(addr, static_cast<int>(bufsz), buf);

    if (verify_ena_.to_bool()) {
        uint8_t *chk = new uint8_t [static_cast<int>(bufsz) + 8];
        itap_->read(addr, static_cast<int>(bufsz), chk);
        for (uint64_t i = 0; i < bufsz; i++) {
            if (buf[i] != chk[i]) {
                RISCV_error("[%08" RV_PRI64 "x] verif. error %02x != %02x",
                            addr + i, buf[i], chk[i]);
            }
        }
        delete [] chk;
    }
    return bufsz;
}

uint64_t ElfLoaderService::initMemory(uint64_t addr, uint64_t bufsz) {
    uint32_t zero = 0;
    uint64_t cnt = 0;
    while (cnt < bufsz) {
        itap_->write(addr, 4, reinterpret_cast<uint8_t *>(&zero));
        addr += 4;
        cnt += 4;
    }
    return bufsz;
}

}  // namespace debugger
