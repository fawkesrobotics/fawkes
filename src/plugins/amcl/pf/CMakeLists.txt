add_library(fawkes_amcl_pf SHARED pf.c pf_kdtree.c pf_pdf.c pf_vector.c eig3.c
                                  pf_draw.c)
target_link_libraries(fawkes_amcl_pf m)
